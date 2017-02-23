--{-# LANGUAGE TypeFamilies #-}
{-|
Module:       Robotics.ArDrone.NavDataTypes
Description:  Types that represent the sensor data sent from the drone
License:      GPL-2
Maintainer:   thomasmhergenroeder@gmail.com
Stability:    Tested on Windows 10

This module holds the representations for the navigational data options the
drone offers. All options are available, from Demo to GPS data. Also the
Checksum is defined here. <br>
The main type that contains all the types is defined here as well. <br>
There are additionally some helper types defined that make the other time more
convenient to handle and read. <br>
The data type of the different fields and their units are not clearly documented
 y Parrot, so I can't make too much assumptions here. For more information on
 the types look at https://github.com/felixge/node-ar-drone and
 ARDrone_SDK_2_0/Examples/Linux/Navigation/Sources/navdata_client/navdata_ihm.c.
-}
module Robotics.ArDrone.NavDataTypes where

import Data.Matrix
import Data.Word
import Data.VectorSpace
import Data.Int

-- | This data type holds the euler angels of the drone
data Eulers = Eulers { phi :: Float
                     , theta :: Float
                     , psi :: Float
                     } deriving (Show)

-- | this is a vector type that is useful, because much of the data is stored in
-- triples that hold information for a common sensor in all three dimensions.
data Vector = Vector { x :: Float
                     , y :: Float
                     , z :: Float
                     } deriving (Show, Read)

instance Monoid Vector where
  mempty = Vector 0 0 0
  Vector x1 y1 z1 `mappend` Vector x2 y2 z2 = Vector (x1 + x2) (y1 + y2) (z1 + z2)

-- | This function converts my custom vector type to the matrix type from the
-- library I used. This makes it easy to multiply the calibration matrix with a
-- vector to lesson the error of measurement.
vectorToMatrix :: Vector -> Matrix Float
vectorToMatrix (Vector x y z) = fromLists [[x],[y],[z]]

-- | Scales the vector.
scaleVector :: Float -> Vector -> Vector
scaleVector f (Vector x y z) = Vector (f*x) (f*y) (f*z)

-- | Datatype holding the important data from the the demo_data struct. Some
-- deprecated information is dropped.
data DemoData = DemoData { flyState :: Word32
                         , batteryPercentage :: Word32
                         , demoTheta :: Float
                         , demoPhi :: Float
                         , demoPsi :: Float
                         , demoAltitude :: Word32
                         , demoVelocity :: Vector
                         } deriving (Show)

-- | This is the navdata package header that is placed before the various
-- options.
data Header = Header { header :: Word32     -- ^ Header to identify the message from the drone.
                     , state :: Word32      -- ^ Information about the state of the drone in a 32 bit field.
                     , sequenceNr :: Word32 -- ^ Sequnce number of this navdata packet.
                     , visionFlag :: Word32 -- ^ vision flag.
                     } deriving (Show)

-- | This holds the drone time in milliseconds
data Time = Time { droneTime :: Word32
                 } deriving (Show)

-- | This data type holds the raw values from the various sensors, so they are
-- not converted to any unit, just the plain bits the ADC produced.
data RawMeasures = RawMeasures { accVector :: (Word16, Word16, Word16)
                                 , gyroVector :: (Word16, Word16, Word16)
                                 , gyro110 :: ( Word16, Word16 )
                                 , batteryMilliVolt :: Word32
                                 , usEchoStart :: Word16
                                 , usEchoEnd :: Word16
                                 , usEchoAssociation :: Word16
                                 , usEchoDistance :: Word16
                                 , usCurveTime :: Word16
                                 , usCurveValue :: Word16
                                 , usCurveRef :: Word16
                                 , echoFlagIni :: Word16
                                 , echoNum :: Word16
                                 , echoSum :: Word32
                                 , altTemp :: Word32
                                 , gradient :: Word16
                                 } deriving (Show)

-- | This represents the physical measurements of the accelerometer and the
-- gyroscope. The accelerometer values are in m/s^2
data PhysMeasures = PhysMeasures { accelerometers :: Vector
                                 , gyroscopes :: Vector
                                 } deriving (Show)

-- | 3 dimensional offsets of the gyroscope.
data GyroOffsets = GyroOffsets { gOffsets :: Vector } deriving (Show)

-- | Euler angles theta and phi for the drone.
data EulerAngles = EulerAngles { eulerTheta :: Float
                               , eulerPhi :: Float
                               } deriving (Show)

-- | References option. Do not know much more about it.
data References = References { refTheta :: Int
                             , refPhi :: Int
                             , refThetaI :: Int
                             , refPhiI :: Int
                             , refPitch :: Int
                             , refRoll :: Int
                             , refYaw :: Int
                             , refPsi :: Int
                             , refVx :: Float
                             , refVy :: Float
                             , refThetaMod :: Float
                             , refPhiMod :: Float
                             , refKVX :: Float
                             , refKVY :: Float
                             , refKMode :: Int
                             , refUiTime :: Float
                             , refUiTheta :: Float
                             , refUiPhi :: Float
                             , refUiPsi :: Float
                             , refUiPsiAccuracy :: Float
                             , refUiSeq :: Int
                             } deriving (Show)

-- | Trims option.
data Trims = Trims { tAngularRatesR :: Float
                   , tEulerTheta :: Float
                   , tEulerPhi :: Float
                   } deriving (Show)

-- | RcReferences.
data RcReferences = RcReferences { rcPitch :: Int
                                 , rcRoll :: Int
                                 , rcYaw :: Int
                                 , rcGaz :: Int
                                 , rcAg :: Int
                                 } deriving (Show)

-- | Pulse width modulation values. DO NOT USE THIS OPTION. Still a weird bug
-- when parsing it.
data Pwm = Pwm { motors :: Word32
               , satMotors :: Word32
               , gazFeedForward :: Float
               , gazAltitude :: Float
               , altitudeIntegral :: Float
               , vzRef :: Float
               , uPitch :: Int
               , uRoll :: Int
               , uYaw :: Int
               , yawUI :: Float
               , uPitchPlanif :: Int
               , uRollPlanif :: Int
               , uYawPlanif :: Int
               , uGazPlanif :: Float
               , motorCurrents :: (Word16, Word16, Word16, Word16)
               , altitudeProp :: Float
               , altitudeDer :: Float
               } deriving (Show)

-- | This data type holds some information about the altitude of the drone.
data Altitude = Altitude { altVision :: Int
                         , altVel :: Float
                         , altRef :: Int
                         , altRaw :: Int
                         , altObsAcc :: Float
                         , altObsAlt :: Float
                         , altObsX :: Vector
                         , altObsState :: Word32
                         , altEstVb :: (Float, Float)
                         , altEstState :: Word32
                         } deriving (Show)

-- | VisionRaw option.
data VisionRaw = VisionRaw { vrTx :: Float
                           , vrTy :: Float
                           , vrTz :: Float
                           } deriving (Show)

-- | VisionOf option.
data VisionOf = VisionOf { voDx :: (Float, Float, Float, Float, Float)
                         , voDy :: (Float, Float, Float, Float, Float)
                         } deriving (Show)

-- | Vision option.
data Vision = Vision { viState :: Word32
                     , viMisc :: Int
                     , viPhiTrim :: Float
                     , viPhiRefProp :: Float
                     , viThetaTrim :: Float
                     , viThetaProp :: Float
                     , newRawPicture :: Int
                     , viEulerAngles :: Eulers
                     , viCaptureAltitude :: Int
                     , viCaptureTime :: Word32
                     , viBodyV :: Vector
                     , viDelta :: Vector
                     , viGoldDefined :: Word32
                     , viGoldReset :: Word32
                     , viGoldX :: Float
                     , viGoldY :: Float
                     } deriving (Show)

-- | VisionPerf option.
data VisionPerf = VisionPerf { vpSzo :: Float
                             , vpCorners :: Float
                             , vpCompute :: Float
                             , vpTracking :: Float
                             , vpTrans :: Float
                             , vpUpdate :: Float
                             , custom :: [Float]
                             } deriving (Show)

-- | TrackersSend option.
data TrackersSend = TrackersSend { locked :: [Int]
                                 , point :: [(Float, Float)]
                                 } deriving (Show)

-- | VisionDetect option.
data VisionDetect = VisionDetect { nbDetected :: Word32
                                 , vdType :: [Word32]
                                 , vdXc :: [Word32]
                                 , vdYc :: [Word32]
                                 , vdWidth :: [Word32]
                                 , vdHeight :: [Word32]
                                 , vdDist :: [Word32]
                                 , vdOrientationAngle :: [Float]
                                 , vdRotation :: [Matrix Float]
                                 , vdTranslation :: [Vector]
                                 , vdCameraSource :: [Word32]
                                 } deriving (Show)

-- | Watchdog option.
data Watchdog = Watchdog { wd :: Word32 } deriving (Show)

-- | AdcDataFrame option.
data AdcDataFrame = AdcDataFrame { adcVersion :: Word32
                                 , adcFrame :: [Word8]
                                 } deriving (Show)

-- | Information about the bottom camera.
data VideoStream = VideoStream { vsQuant :: Word8
                               , vsFrame :: (Word32, Word32)
                               , vsAtcmd :: (Word32, Word32, Float, Word32)
                               , vsBitrate :: (Word32, Word32)
                               , vsData :: [Int]
                               , tcpQueueLevel :: Word32
                               , fifoQueueLevel :: Word32
                               } deriving (Show)

-- | Information about augemented reality games
data Games = Games { counters :: (Word32, Word32) } deriving (Show)

-- | Raw pressure data from the sensors ADC.
data PressureRaw = PressureRaw { prUp :: Int
                               , prUt :: Word16
                               , prTemperature :: Int16
                               , prPressure :: Int
                               } deriving (Show)

-- | Information about the magnetic field of the earth to determine the
-- direction of the drone.
data Magneto = Magneto { magMx :: Int16
                       , magMy :: Int16
                       , magMz :: Int16
                       , magRaw :: Vector
                       , magRectified :: Vector
                       , magOffset :: Vector
                       , magHeading :: (Float, Float, Float)
                       , magOk :: Word8
                       , magState :: Word32
                       , magRadius :: Float
                       , error :: (Float, Float)
                       } deriving (Show)

-- | Information about the windspeed and angle.
data Windspeed = Windspeed { wsSpeed :: Float
                           , wsAngle :: Float
                           , wsCompensation :: (Float, Float)
                           , wsStateX :: [Float]
                           , wsDebug :: [Float]
                           } deriving (Show)

-- | KalmanPressure option.
data KalmanPressure = KalmanPressure { kpOffsetPressure :: Float
                                     , kpAltitude :: Float
                                     , kpVelocity :: Float
                                     , kpAngle :: (Float, Float)
                                     , kpUs :: (Float, Float)
                                     , kpCovariance :: (Float, Float, Float)
                                     , kpGroundEffect :: Word8
                                     , kpSum :: Float
                                     , kpReject :: Word8
                                     , kpUMultisinus :: Float
                                     , kpGazAltitude :: Float
                                     , kpFlagMultisinus :: Word8
                                     , kpFlagMultisinusStart :: Word8
                                     } deriving (Show)

-- | Information about the video stream from the front camera.
data HDVideoStream = HDVideoStream { hdState :: Word32
                                   , hdStorage :: (Word32, Word32)
                                   , hdUsbkey :: (Word32, Word32)
                                   , hdFrameNumber :: Word32
                                   , hdUsbRemainingTime :: Word32
                                   } deriving (Show)

-- | Option that informs about the Wifi quality.
data Wifi = Wifi { wifiLinkQuality :: Float } deriving (Show)

-- | SatChannel option
data SatChannel = SatChannel { scSat :: Word8
                             , scCn0 :: Word8
                             } deriving (Show)

-- | The complete set of GPS data, if you have a GPS chip on your drone.
data Gps = Gps { latitude :: Double
               , longitude :: Double
               , elevation :: Double
               , hdop :: Double
               , gpsDataAvailable :: Int
               , gpsZeroValidated :: Int
               , gpsWptValidated :: Int
               , lat0 :: Double
               , lon0 :: Double
               , latFuse :: Double
               , lonFuse :: Double
               , gpsState :: Word32
               , xTraj :: Float
               , xRef :: Float
               , yTraj :: Float
               , yRef :: Float
               , thetaP :: Float
               , phiP :: Float
               , thetaI :: Float
               , phiI :: Float
               , thetaD :: Float
               , phiD :: Float
               , vdop :: Double
               , pdop :: Double
               , gpsSpeed :: Float
               , lastFrameTimestamp :: Word32
               , gpsDegreee :: Float
               , gpsDegreeMag :: Float
               , gpsEhpe :: Float
               , gpsEhve :: Float
               , c_n0 :: Float
               , nbSatellites :: Word32
               , gpsChannels :: [SatChannel]
               , gpsPlugged :: Int
               , gpsEphemerisStatus :: Word32
               , gpsVxTraj :: Float
               , gpsVyTraj :: Float
               , gpsFirmwareStatus :: Word32
               } deriving (Show)

-- | CheckSUm of the current packet.
data CheckSum = CheckSum { value :: Word32 } deriving (Show)

-- | Wrapping data type that holds all the data types for every option. They are
-- all wrapped in Maybe, since the option might be not present due to the
-- navdata configuration.
data NavData = NavData { navDataHeader :: Maybe Header
                       , demoData :: Maybe DemoData
                       , time :: Maybe Time
                       , rawMeasures :: Maybe RawMeasures
                       , physMeasures :: Maybe PhysMeasures
                       , gyroOffsets :: Maybe GyroOffsets
                       , eulerAngles :: Maybe EulerAngles
                       , references :: Maybe References
                       , trims :: Maybe Trims
                       , rcReferences :: Maybe RcReferences
                       , pwm :: Maybe Pwm
                       , altitude :: Maybe Altitude
                       , visionRaw :: Maybe VisionRaw
                       , visionOf :: Maybe VisionOf
                       , vision :: Maybe Vision
                       , visionPerf :: Maybe VisionPerf
                       , trackersSend :: Maybe TrackersSend
                       , visionDetect :: Maybe VisionDetect
                       , watchdog :: Maybe Watchdog
                       , adcDataFrame :: Maybe AdcDataFrame
                       , videoStream :: Maybe VideoStream
                       , games :: Maybe Games
                       , pressureRaw :: Maybe PressureRaw
                       , magneto :: Maybe Magneto
                       , windspeed :: Maybe Windspeed
                       , kalmanPressure :: Maybe KalmanPressure
                       , hdVideoStream :: Maybe HDVideoStream
                       , wifi :: Maybe Wifi
                       , gps :: Maybe Gps
                       , checkSum :: Maybe CheckSum
                       } deriving (Show)

-- | Returns an unitialized Navdata value
emptyNavData :: NavData
emptyNavData = NavData Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing
