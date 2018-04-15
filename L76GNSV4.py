# quectel L76 GNSS library for Micropython
# pycom pytrack module for wipy, lopy, sipy ...
# Andre Peeters
# andre@andrethemac.be
# v4 2018-03-24
# v4b 2018-03-26 faster fix using GLL instead of GGA
# based upon the original L76GLNSS library
# and the modifications by neuromystix
# every lookup of coordinates or other GPS data has to wait for the
# right GPS message, no caching of GPS data
# MIT licence

from machine import Timer
import time
import gc
import binascii


class L76GNSS:

    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=180,debug=False):
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
        self.debug = debug

    def _read(self):
        """read the data stream form the gps"""
        # Changed from 64 to 128 - I2C L76 says it can read till 255 bytes
        self.reg = self.i2c.readfrom(GPS_I2CADDR, 128)
        return self.reg

    @staticmethod
    def _convert_coord(coord, orientation):
        """convert a ddmm.mmmm to dd.dddddd degrees"""
        coord = (float(coord) // 100) + ((float(coord) % 100) / 60)
        if orientation == 'S' or orientation == 'W':
            coord *= -1
        return coord

    def _mixhash(self, keywords, sentence):
        """return hash with keywords filled with sentence"""
        ret = {}
        if len(keywords) == len(sentence):
            for k, s in zip(keywords, sentence):
                ret[k] = s
            try:
                ret['Latitude'] = self._convert_coord(ret['Latitude'], ret['NS'])
            except:
                pass
            try:
                ret['Longitude'] = self._convert_coord(ret['Longitude'], ret['EW'])
            except:
                pass
        return ret

    def _GGA(self, sentence):
        """essentials fix and accuracy data"""
        keywords = ['NMEA', 'UTCTime', 'Latitude', 'NS', 'Longitude', 'EW',
                    'FixStatus', 'NumberOfSV', 'HDOP',
                    'Altitude', 'M', 'GeoIDSeparation', 'M', 'DGPSAge', 'DGPSStationID']
        return self._mixhash(keywords, sentence)

    def _GLL(self, sentence):
        """hash a GLL sentence (geolocation)"""
        keywords = ['NMEA', 'Latitude', 'NS', 'Longitude', 'EW',
                    'UTCTime', 'dataValid', 'PositioningMode']
        return self._mixhash(keywords, sentence)

    def _RMC(self, sentence):
        """required minimum position data"""
        if len(sentence) == 11:
            sentence.append('N')
        keywords = ['NMEA', 'UTCTime', 'dataValid', 'Latitude', 'NS', 'Longitude', 'EW',
                    'Speed', 'COG', 'Date', '', '', 'PositioningFix']
        return self._mixhash(keywords, sentence)

    def _VTG(self, sentence):
        """track and ground speed"""
        keywords = ['NMEA', 'COG-T', 'T', 'COG-M', 'M', 'SpeedKnots', 'N', 'SpeedKm', 'K',
                    'PositioningFix']
        return self._mixhash(keywords, sentence)

    def _GSA(self, sentence):
        keywords = ['NMEA', 'Mode', 'FixStatus',
                    'SatelliteUsed01', 'SatelliteUsed02', 'SatelliteUsed03',
                    'SatelliteUsed04', 'SatelliteUsed05', 'SatelliteUsed06',
                    'SatelliteUsed07', 'SatelliteUsed08', 'SatelliteUsed09',
                    'SatelliteUsed10', 'SatelliteUsed11', 'SatelliteUsed12',
                    'PDOP', 'HDOP', 'VDOP']
        return self._mixhash(keywords, sentence)

    def _GSV(self, sentence):
        keywords = ['NMEA', 'NofMessage', 'SequenceNr', 'SatellitesInView',
                    'SatelliteID1', 'Elevation1', 'Azimuth1', 'SNR1',
                    'SatelliteID2', 'Elevation2', 'Azimuth2', 'SNR2',
                    'SatelliteID3', 'Elevation3', 'Azimuth3', 'SNR3',
                    'SatelliteID4', 'Elevation4', 'Azimuth4', 'SNR4']
        return self._mixhash(keywords, sentence)

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
        return None

    def _read_message(self, messagetype='GLL', timeout=None, debug=False):
        """reads output from the GPS and translates it to a message"""
        if timeout is None:
            timeout = self.timeout
        self.chrono.reset()
        self.chrono.start()
        messagetype = messagetype[-3:]
        nmea = b''
        chrono_running = True

        while chrono_running:
            nmea += self._read().strip(b'\r\n')
            start = nmea.find(b'$')
            if start > 0:
                nmea = nmea[start:]
                end = nmea.find(b'*')
                if end > 0:
                    nmea = nmea[:end+3].decode('utf-8')
                    if debug:
                        if nmea is not None:
                            print(self.fix, timeout - self.chrono.read(), nmea[1:-3])
                    nmea_message = self._decodeNMEA(nmea, debug=debug)
                    if nmea_message is not None:
                        if not self.fix:
                            try:
                                if (nmea_message['NMEA'] == 'GLL' and nmea_message['PositioningMode'] != 'N') \
                                        or (nmea_message['NMEA'] == 'GGA' and int(nmea_message['FixStatus']) >= 1):
                                    self.fix = True
                            except:
                                pass
                        else:
                            try:
                                if nmea_message['NMEA'] == messagetype or messagetype is None:
                                    chrono_running = False
                            except:
                                pass
                    nmea = b''
                    gc.collect()
            if start <= 0 or len(nmea) > 82:
                nmea = b''
                gc.collect()
            if timeout is not None and self.chrono.read() > timeout:
                chrono_running = False
                nmea_message = None
        self.chrono.stop()
        if debug:
            print("fix in", self.chrono.read(), "seconds")
        if not self.fix:
            return None
        return nmea_message

    def fixed(self):
        """fixed yet?"""
        return self.fix

    def get_fix(self, debug=False):
        """look for a fix"""
        self._read_message('GLL', debug=debug)
        return self.fix

    def gps_message(self, messagetype=None, timeout=None, debug=False):
        """returns the last message from the L76 gps"""
        return self._read_message(messagetype=messagetype, timeout=timeout, debug=debug)

    def coordinates(self, timeout=None, debug=False):
        """you are here"""
        msg, latitude, longitude = None, None, None
        msg = self._read_message('GLL', timeout=timeout, debug=debug)
        if msg is not None:
            latitude = msg['Latitude']
            longitude = msg['Longitude']
        return latitude, longitude

    def get_speed_RMC(self, timeout=10):
        """returns your speed and direction as return by the ..RMC message"""
        msg, speed, COG = None, None, None
        msg = self._read_message('RMC', timeout=timeout)
        if msg is not None:
            speed = msg['Speed']
            COG = msg['COG']
        return speed, COG

    def get_speed(self, timeout=10):
        """returns your speed and direction in degrees"""
        msg, speed, COG = None, None, None
        msg = self._read_message('VTG', timeout=timeout)
        if msg is not None:
            speed = msg['SpeedKm']
            COG = msg['COG-T']
        return speed, COG

    def get_location(self, MSL=False, timeout=None):
        """location, altitude and HDOP"""
        msg, latitude, longitude, HDOP, altitude = None, None, None, None, None
        msg = self._read_message('GGA', timeout=timeout)
        if msg is not None:
            latitude = msg['Latitude']
            longitude = msg['Longitude']
            HDOP = msg['HDOP']
            if MSL:
                altitude = msg['Altitude']
            else:
                altitude = msg['GeoIDSeparation']
        return latitude, longitude, HDOP, altitude

    def getUTCTime(self, timeout=None, debug=False):
        """return UTC time or None when nothing if found"""
        msg = self._read_message('GLL', timeout=timeout, debug=debug)
        if msg is not None:
            utc_time = msg['UTCTime']
            return "{}:{}:{}".format(utc_time[0:2], utc_time[2:4], utc_time[4:6])
        else:
            return None

    def getUTCDateTime(self, timeout=None,debug=False):
        """return UTC date time or None when nothing if found"""
        msg = self._read_message('RMC', timeout=timeout, debug=debug)
        if msg is not None:
            utc_time = msg['UTCTime']
            utc_date = msg['Date']
            return "20{}-{}-{}T{}:{}:{}+00:00".format(utc_date[0:2], utc_date[2:4], utc_date[4:6],
                                                      utc_time[0:2], utc_time[2:4], utc_time[4:6])
        else:
            return None
