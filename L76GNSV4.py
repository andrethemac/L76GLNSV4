# quectel L76 GNSS library for Micropython
# pycom pytrack module for wipy, lopy, sipy ...
# Andre Peeters
# andre@andrethemac.be
# v4 2018-03-24
# v4b 2018-03-26 faster fix using GLL instead of GGA
# v5 2019-06-07 added pmtk commands
# based upon the original L76GLNSS library
# and the modifications by neuromystix
# every lookup of coordinates or other GPS data has to wait for the
# right GPS message, no caching of GPS data
# MIT licence

from machine import Timer
import time
import gc
import binascii

# TODO: annotate sattelites in view


class L76GNSS:

    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=180, debug=False):
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.chrono = Timer.Chrono()
        self.timeout = timeout
        self.timeout_status = True
        self.reg = bytearray(1)
        self.i2c.writeto(GPS_I2CADDR, self.reg)
        self.fix = False
        self.Latitude = None
        self.Longitude = None
        self.debug = debug
        self.timeLastFix = 0
        self.ttf = -1
        self.lastmessage = {}

    def _read(self):
        """read the data stream form the gps"""
        # Changed from 64 to 128 - I2C L76 says it can read till 255 bytes
        reg = b''
        try:
            reg = self.i2c.readfrom(GPS_I2CADDR, 255)
        except:
            pass
        return reg

    @staticmethod
    def _convert_coord(coord, orientation):
        """convert a ddmm.mmmm to dd.dddddd degrees"""
        coord = (float(coord) // 100) + ((float(coord) % 100) / 60)
        if orientation == 'S' or orientation == 'W':
            coord *= -1
        return coord

    def time_fixed(self):
        """how long till the last fix"""
        return int(time.ticks_ms()/1000) - self.timeLastFix

    def _mixhash(self, keywords, sentence):
        """return hash with keywords filled with sentence"""
        ret = {}
        while len(keywords) - len(sentence) > 0:
            sentence += ('',)
        if len(keywords) == len(sentence):
            # for k, s in zip(keywords, sentence):
            #     ret[k] = s
            ret = dict(zip(keywords, sentence))
            try:
                ret['Latitude'] = self._convert_coord(ret['Latitude'], ret['NS'])
            except:
                pass
            try:
                ret['Longitude'] = self._convert_coord(ret['Longitude'], ret['EW'])
            except:
                pass
            return ret
        else:
            return None

    def _GGA(self, sentence):
        """essentials fix and accuracy data"""
        keywords = ['NMEA', 'UTCTime', 'Latitude', 'NS', 'Longitude', 'EW',
                    'FixStatus', 'NumberOfSV', 'HDOP',
                    'Altitude', 'M', 'GeoIDSeparation', 'M', 'DGPSAge', 'DGPSStationID']
        return self._mixhash(keywords, sentence)

    def _GLL(self, sentence):
        """GLL sentence (geolocation)"""
        keywords = ['NMEA', 'Latitude', 'NS', 'Longitude', 'EW',
                    'UTCTime', 'dataValid', 'PositioningMode']
        return self._mixhash(keywords, sentence)

    def _RMC(self, sentence):
        """required minimum position data"""
        if len(sentence) == 11:
            sentence.append('N')
        keywords = ['NMEA', 'UTCTime', 'dataValid', 'Latitude', 'NS', 'Longitude',
                    'EW', 'Speed', 'COG', 'Date', '', '', 'PositioningMode',
                    "NavigationalStatus"]
        return self._mixhash(keywords, sentence)

    def _VTG(self, sentence):
        """track and ground speed"""
        keywords = ['NMEA', 'COG-T', 'T', 'COG-M', 'M', 'SpeedKnots', 'N', 'SpeedKm', 'K',
                    'PositioningMode']
        return self._mixhash(keywords, sentence)

    def _GSA(self, sentence):
        """fix state, the sattelites used and DOP info"""
        keywords = ['NMEA', 'Mode', 'FixStatus',
                    'SatelliteUsed01', 'SatelliteUsed02', 'SatelliteUsed03',
                    'SatelliteUsed04', 'SatelliteUsed05', 'SatelliteUsed06',
                    'SatelliteUsed07', 'SatelliteUsed08', 'SatelliteUsed09',
                    'SatelliteUsed10', 'SatelliteUsed11', 'SatelliteUsed12',
                    'PDOP', 'HDOP', 'VDOP', "GNSSSystemId"]
        return self._mixhash(keywords, sentence)

    def _GSV(self, sentence):
        """four of the sattelites seen"""
        keywords = ['NMEA', 'NofMessage', 'SequenceNr', 'SatellitesInView',
                    'SatelliteID1', 'Elevation1', 'Azimuth1', 'SNR1',
                    'SatelliteID2', 'Elevation2', 'Azimuth2', 'SNR2',
                    'SatelliteID3', 'Elevation3', 'Azimuth3', 'SNR3',
                    'SatelliteID4', 'Elevation4', 'Azimuth4', 'SNR4',
                    "SignalId"]
        return self._mixhash(keywords, sentence)

    def _pmtkAck(self, sentence):
        keywords = ['PMTK', 'command', 'response']
        return self._mixhash(keywords, sentence)
        # if sentence[2] == 3:
        #     return True
        # else:
        #     return False

    def _decodeNMEA(self, nmea, debug=False):
        """turns a message into a hash"""
        nmea_sentence = nmea[:-3].split(',')
        sentence = nmea_sentence[0][3:]
        nmea_sentence[0] = sentence
        if debug:
            print(sentence, "->", nmea_sentence)
        if sentence == 'RMC':
            return self._RMC(nmea_sentence)
        if sentence == 'VTG':
            return self._VTG(nmea_sentence)
        if sentence == 'GGA':
            return self._GGA(nmea_sentence)
        if sentence == 'GSA':
            return self._GSA(nmea_sentence)
        if sentence == 'GSV':
            return self._GSV(nmea_sentence)
        if sentence == 'GLL':
            return self._GLL(nmea_sentence)
        if sentence == '001':
            return self._pmtkAck(nmea_sentence)
        return None

    def _read_message(self, messagetype='GLL', debug=False):
        if type(messagetype) == type(()):
            mt = []
            for m in messagetype:
                mt += [m[-3:]]
            messagetype = mt
        else:
            messagetype = messagetype[-3:]
        if debug:
            print("messagetype", messagetype)
        messagefound = False
        while not messagefound:
            nmea = self._read_message_raw(debug=debug)
            nmea_message = self._decodeNMEA(nmea, debug=debug)
            if debug:
                print("nmea_message", nmea_message)
            if nmea_message is not None:
                messagefound = (nmea_message['NMEA'] in messagetype)
                if debug:
                    print("found message?", messagefound)
        self.lastmessage = nmea_message
        return nmea_message

    def _read_message_raw(self, debug=False):
        """reads output from the GPS and translates it to a message"""
        nmea = b''
        start = nmea.find(b'$')
        while start < 0:
            nmea += self._read() #.strip(b'\r\n')
            start = nmea.find(b'$')
        if debug:
            print("nmea raw", len(nmea), start, nmea)
        nmea = nmea[start:]
        # end = nmea.find(b'*')
        end = nmea.find(b'\r\n')
        while end < 0:
            nmea += self._read() #.strip(b'\r\n')
            # end = nmea.find(b'*')
            end = nmea.find(b'\r\n')
        nmea = nmea[:end+3].decode('utf-8')
        nmea = nmea[:-3]
        nmea = nmea.replace('\n','')
        if debug:
            if nmea is not None:
                print("nmea raw fix", self.fix, len(nmea), nmea)
        gc.collect()
        return nmea

    def fixed(self):
        """fixed yet? returns true or false"""
        nmea_message = self.lastmessage
        pm = fs = False
        if nmea_message != {}:
            if nmea_message['NMEA'] in ('RMC', 'GLL'):  # 'VTG',
                pm = nmea_message['PositioningMode'] != 'N'
            if nmea_message['NMEA'] in ('GGA',):  # 'GSA'
                fs = int(nmea_message['FixStatus']) >= 1
        if pm or fs:
            self.fix = True
            self.timeLastFix = int(time.ticks_ms() / 1000)
            self.Latitude = nmea_message['Latitude']
            self.Longitude = nmea_message['Longitude']
        else:
            self.fix = False
            self.timeLastFix = 0xffffffff
            self.Latitude = None
            self.Longitude = None
            self.ttf = -1
        return self.fix

    def get_fix(self, force=True, debug=False, timeout=None):
        """look for a fix, use force to refix, returns true or false"""
        if force:
            self.fix = False
        if timeout is None:
            timeout = self.timeout
        self.chrono.reset()
        self.chrono.start()
        chrono_running = True

        while chrono_running and not self.fix:
            nmea_message = self._read_message(('RMC', 'VTG', 'GLL', 'GGA', 'GSA'), debug=debug)
            if nmea_message is not None:
                pm = fs = False
                try:
                    if nmea_message['NMEA'] in ('RMC', 'GLL'):  #'VTG',
                        pm = nmea_message['PositioningMode'] != 'N'
                    if nmea_message['NMEA'] in ('GGA', ):  #'GSA'
                        fs = int(nmea_message['FixStatus']) >= 1
                    if pm or fs:
                        self.chrono.stop()
                        self.fix = True
                        self.timeLastFix = int(time.ticks_ms() / 1000) - self.timeLastFix
                        self.ttf = round(self.chrono.read())
                        self.Latitude = nmea_message['Latitude']
                        self.Longitude = nmea_message['Longitude']
                except:
                    pass
            if self.chrono.read() > timeout:
                chrono_running = False
        self.chrono.stop()
        if debug:
            print("fix in", self.chrono.read(), "seconds")
        return self.fix

    def gps_message(self, messagetype=None, debug=False):
        """returns the last message from the L76 gps"""
        return self._read_message(messagetype=messagetype, debug=debug)

    def coordinates(self, debug=False):
        """you are here"""
        msg, latitude, longitude = None, None, None
        if not self.fix:
            self.get_fix(debug=debug)
        msg = self._read_message(('RMC', 'GGA', 'GLL'), debug=debug)
        if msg is not None:
            self.Latitude = msg['Latitude']
            self.Longitude = msg['Longitude']
        return dict(latitude=self.Latitude, longitude=self.Longitude, ttf=self.ttf)

    def get_speed_RMC(self):
        """returns your speed and direction as return by the ..RMC message"""
        msg, speed, COG = None, None, None
        msg = self._read_message(messagetype='RMC')
        if msg is not None:
            speed = msg['Speed']
            COG = msg['COG']
        return dict(speed=speed, COG=COG)

    def get_speed(self):
        """returns your speed and direction in degrees"""
        msg, speed, COG = None, None, None
        msg = self._read_message(messagetype='VTG')
        if msg is not None:
            speed = msg['SpeedKm']
            COG = msg['COG-T']
        return dict(speed=speed, COG=COG)

    def get_location(self, MSL=False,debug=False):
        """location, altitude and HDOP"""
        msg, latitude, longitude, HDOP, altitude = None, None, None, None, None
        if not self.fix:
            self.get_fix(debug=debug)
        msg = self._read_message(messagetype='GGA')
        if msg is not None:
            latitude = msg['Latitude']
            longitude = msg['Longitude']
            HDOP = msg['HDOP']
            if MSL:
                altitude = msg['GeoIDSeparation']
            else:
                altitude = msg['Altitude']
        return dict(latitude=latitude, longitude=longitude, HDOP=HDOP, altitude=altitude, ttf=self.ttf)

    def getUTCTime(self, debug=False):
        """return UTC time or None when nothing if found"""
        msg = self._read_message(('GLL','RMC','GGA'), debug=debug)
        if msg is not None:
            utc_time = msg['UTCTime']
            return "{}:{}:{}".format(utc_time[0:2], utc_time[2:4], utc_time[4:6])
        else:
            return None

    def getUTCDateTime(self, debug=False):
        """return UTC date time or None when nothing if found"""
        msg = self._read_message(messagetype='RMC', debug=debug)
        if msg is not None:
            utc_time = msg['UTCTime']
            utc_date = msg['Date']
            if str(utc_date)[-2:] == '80':
                return None
            return "20{}-{}-{}T{}:{}:{}+00:00".format(utc_date[4:6], utc_date[2:4], utc_date[0:2],
                                                      utc_time[0:2], utc_time[2:4], utc_time[4:6])
        else:
            return None

    def getUTCDateTimeTuple(self, debug=False):
        """return UTC date time or None when nothing if found"""
        msg = self._read_message(messagetype='RMC', debug=debug)
        if msg is not None:
            utc_time = msg['UTCTime']
            utc_date = msg['Date']
            if debug:
                print('utc_date type: %s' % type(utc_date))
            if str(utc_date)[-2:] == '80':
                return None
            year = '20'
            year += utc_date[4:6]
            return (int(year), int(utc_date[2:4]), int(utc_date[0:2]), int(utc_time[0:2]), int(utc_time[2:4]), int(utc_time[4:6]))
        else:
            return None

    def enterStandBy(self, debug=False):
        """ standby mode, needs powercycle to restart"""
        message = bytearray('$PMTK161,0*28\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)

    def hotStart(self, debug=False):
        """ HotStart the receiver, using data in nv store"""
        message = bytearray('$PMTK101*32\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def warmStart(self, debug=False):
        """ warmStart the receiver, not using data in nv store, using last know messages"""
        message = bytearray('$PMTK102*31\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def coldStart(self, debug=False):
        """ coldStart the receiver, not using any data """
        message = bytearray('$PMTK103*30\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def fullColdStart(self, debug=False):
        """ full cold start the receiver, as cold start as in powercycle"""
        message = bytearray('$PMTK104*37\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def setPeriodicMode(self, mode=0,
                        runtime=1000, sleeptime=1000,
                        secruntime=10000, secsleeptime=10000, debug=False):
        """
        mode :
            0 : fully on
            1 : periodic backup
            2 : periodic standby
            4 : perpetual standy by => needs powercycle to start the gps
            8 : allways locate standby
            9 : allways locate backup
        runtime: time the unit is fully operational
        sleeptime: time the unit is in standy/backup modus
        secruntime: time the unit is fully operation if the first runtime doesn't get a fix
        secsleeptime: time the unit is in standy/backup modus if the first runtime doesn't get a fix
        """
        if mode in (0, 1, 2, 8, 9):
            message = 'PMTK225,{},{},{},{},{}'.format(mode,runtime, sleeptime, secruntime, secsleeptime)
            checksum = self._get_checksum(message)
            message = bytearray('${}*{}\r\n'.format(message, checksum))
            if debug:
                print("setPeriodicMode",message)
            self.i2c.writeto(GPS_I2CADDR, message)
        # return self._read_message(messagetype='001', debug=debug)

    def setAlwaysOn(self, debug=False):
        self.setPeriodicMode(mode=0)

    def setAlwaysLocateMode(self, mode=8, debug=False):
        if mode in (8, 9):
            message = 'PMTK225,{}'.format(mode)
            checksum = self._get_checksum(message)
            message = bytearray('${}*{}\r\n'.format(message, checksum))
            if debug:
                print("setAlwaysLocateMode",message)
            self.i2c.writeto(GPS_I2CADDR, message)
            return True
            # response = self._read_message(messagetype='001', debug=debug)
            # if debug:
            #     print("response",response)
            # if response['response'] == 3:
            #     return True
            # else:
            #     return False

    def _get_checksum(self, message):
        """calculates the checksum"""
        mc = ord(message[0])
        for m in message[1:]:
            mc = mc ^ ord(m)
        return '{:x}'.format(mc).upper()

    def _check_checksum(self, message):
        """check the checksum of the message"""
        message = message[1:]
        message, checksum = message.split('*')
        return self._get_checksum(message) == checksum
