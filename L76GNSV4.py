# quectel L76 GNSS library for Micropython
# pycom pytrack module for wipy, lopy, sipy ...
# Andre Peeters
# andre@andrethemac.be
# v4 2018-03-24
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

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=None):
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


    def _read(self):
        """read the data stream form the gps"""
        self.reg = self.i2c.readfrom(GPS_I2CADDR, 128)      #Changed from 64 to 128 - I2C L76 says it can read till 255 bytes
        return self.reg


    def _convert_coord(self,coord,orientation):
        """convert a ddmm.mmmm to dd.dddddd degrees"""
        coord_d = (float(coord) // 100) + ((float(coord) % 100) / 60)
        if orientation == 'S' or orientation == 'W':
            coord_d *= -1
        return coord_d


    def _mixHash(self,keywords,sentence):
        """return hash with keywords filled with sentence"""
        ret = {}
        if len(keywords) == len(sentence):
            for k,s in zip(keywords,sentence):
                ret[k] = s
            try:
                ret['Latitude'] = self._convert_coord(ret['Latitude'],ret['NS'])
            except:
                pass
            try:
                ret['Longitude'] = self._convert_coord(ret['Longitude'],ret['EW'])
            except:
                pass
        return ret


    def _GGA(self,sentence):
        keywords = ['NMEA','UTCTime','Latitude','NS','Longitude',
                    'EW','FixStatus','NumberOfSV','HDOP',
                    'Altitude','M','GeoIDSeparation','M','DGPSAge','DGPSStationID']
        return(self._mixHash(keywords,sentence))


    def _GLL(self,sentence):
        keywords = ['NMEA','Latitude','NS','Longitude','EW',
                    'UTCTime','dataValid','PositioningMode']
        return(self._mixHash(keywords,sentence))


    def _RMC(self,sentence):
        if len(sentence) == 11:
            sentence.append('N')
        keywords = ['NMEA','UTCTime','dataValid','Latitude','NS','Longitude',
                'EW','Speed','COG','Date','','','PositioningFix']
        return(self._mixHash(keywords,sentence))


    def _VTG(self,sentence):
        keywords = ['NMEA','COG-T','T','COG-M','M','SpeedKnots','N','SpeedKm',
                'K','PositioningFix']
        return(self._mixHash(keywords,sentence))


    def _GSA(self,sentence):
        keywords = ['NMEA','Mode','FixStatus',
                    'SatelliteUsed01','SatelliteUsed02','SatelliteUsed03',
                    'SatelliteUsed04','SatelliteUsed05','SatelliteUsed06',
                    'SatelliteUsed07','SatelliteUsed08','SatelliteUsed09',
                    'SatelliteUsed10','SatelliteUsed11','SatelliteUsed12',
                    'PDOP','HDOP','VDOP']
        return(self._mixHash(keywords,sentence))


    def _GSV(self,sentence):
        keywords = ['NMEA','NofMessage','SequenceNr','SatellitesInView',
                    'SatelliteID1','Elevation1','Azimuth1','SNR1',
                    'SatelliteID2','Elevation2','Azimuth2','SNR2',
                    'SatelliteID3','Elevation3','Azimuth3','SNR3',
                    'SatelliteID4','Elevation4','Azimuth4','SNR4']
        return(self._mixHash(keywords,sentence))


    def _decodeNMEA(self,nmea):
        nmeaSentence = nmea[:-3].split(',')
        sentence = nmeaSentence[0][3:]
        nmeaSentence[0] = sentence
        if sentence == 'RMC':
            return(self._RMC(nmeaSentence))
        if sentence == 'VTG':
            return(self._VTG(nmeaSentence))
        if sentence == 'GGA':
            return(self._GGA(nmeaSentence))
        if sentence == 'GSA':
            return(self._GSA(nmeaSentence))
        if sentence == 'GSV':
            return(self._GSV(nmeaSentence))
        if sentence == 'GLL':
            return(self._GLL(nmeaSentence))
        return None

    def _readMessage(self,messagetype='GGA',timeout=None):
        if timeout is None:
            timeout = self.timeout
        self.chrono.reset()
        self.chrono.start()
        nmea = b''
        running = True

        while running:
            nmea += self._read().strip(b'\r\n')
            start = nmea.find(b'$')
            if start > 0:
                nmea = nmea[start:]
                end = nmea.find(b'*')
                if end > 0:
                    nmea = nmea[:end+3].decode('utf-8')
                    nmeaMessage = self._decodeNMEA(nmea)
                    if nmeaMessage is not None:
                        if not self.fix:
                            try:
                                if nmeaMessage['NMEA'] == 'GGA' and int(nmeaMessage['FixStatus']) >= 1:
                                    self.fix = True
                            except:
                                pass
                        else:
                            try:
                                if nmeaMessage['NMEA'] == messagetype or messagetype is None:
                                    running = False
                            except:
                                pass
                    nmea = b''
                    gc.collect()
            if start <= 0 or len(nmea) > 82:
                nmea = b''
                gc.collect()
            if timeout is not None and self.chrono.read() > timeout:
                running = False
                nmeaMessage = None
        self.chrono.stop()
        if not self.fix:
            nmeaMessage = None
        return nmeaMessage


    def getFix(self):
        self._readMessage('GGA')
        return self.fix


    def coordinates(self,timeout=None):
        msg, Latitude, Longitude = None, None, None
        msg = self._readMessage('GGA',timeout=timeout);
        if msg is not None:
            Latitude = msg['Latitude']
            Longitude = msg['Longitude']
        return(Latitude,Longitude)


    def getSpeedRMC(self,timeout=10):
        msg, speed,COG = None,None,None
        msg = self._readMessage('RMC',timeout=timeout)
        if msg is not None:
            speed = msg['Speed']
            COG = msg['COG']
        return (speed,COG)


    def getSpeed(self,timeout=10):
        msg, speed,COG = None,None,None
        msg = self._readMessage('VTG',timeout=timeout)
        if msg is not None:
            speed = msg['SpeedKm']
            COG = msg['COG-T']
        return (speed,COG)


    def getLocation(self,MSL=False,timeout=None):
        msg, Latitude, Longitude, HDOP, Altitude = None, None, None, None, None
        msg = self._readMessage('GGA',timeout=timeout);
        if msg is not None:
            Latitude = msg['Latitude']
            Longitude = msg['Longitude']
            HDOP = msg['HDOP']
            if MSL:
                Altitude = msg['Altitude']
            else:
                Altitude = msg['GeoIDSeparation']
        return(Latitude,Longitude, HDOP, Altitude)


    def gpsMessage(self,messagetype=None,timeout=None):
        return self._readMessage(messagetype=messagetype,timeout=timeout)


    def getUTCTime(self,timeout=None):
        """return UTC time or None when nothing if found"""
        msg, UTCTime = None, None
        msg = self._readMessage('GGA',timeout=timeout);
        if msg is not None:
            UTCTime = msg['UTCTime']
            return "{}:{}:{}".format(UTCTime[0:2],UTCTime[2:4],UTCTime[4:6])
        else:
            return None

    def getUTCDateTime(self,timeout=None):
        """return UTC date time or None when nothing if found"""
        msg, UTCTime, UTCDate = None,None,None
        msg = self._readMessage('RMC',timeout=timeout);
        if msg is not None:
            UTCTime = msg['UTCTime']
            UTCDate = msg['Date']
            return "20{}-{}-{}T{}:{}:{}+00:00".format(UTCDate[0:2],UTCDate[2:4],UTCDate[4:6],UTCTime[0:2],UTCTime[2:4],UTCTime[4:6])
        else:
            return None
