--{-# LANGUAGE TypeFamilies #-}

module Robotics.ArDrone.NavDataTypes where

import Data.Matrix
import Data.Word
import Data.VectorSpace
import Data.Int

data Eulers = Eulers { phi :: Float
                     , theta :: Float
                     , psi :: Float
                     } deriving (Show)

data Vector = Vector { x :: Float
                     , y :: Float
                     , z :: Float
                     } deriving (Show, Read)

--instance RModule Vector where
--  type Groundring Vector = Float
--  zeroVector = Vector 0 0 0
--  (*^) n (Vector a b c) = Vector (n*a) (n*b) (n*c)
--  (^+^) (Vector a b c) (Vector d e f) = Vector (a+d) (b+e) (c+f)

--instance VectorSpace Vector where
--

instance Monoid Vector where
  mempty = Vector 0 0 0
  Vector x1 y1 z1 `mappend` Vector x2 y2 z2 = Vector (x1 + x2) (y1 + y2) (z1 + z2)

vectorToMatrix :: Vector -> Matrix Float
vectorToMatrix (Vector x y z) = fromLists [[x],[y],[z]]

--Datatype holding the important data from the the demo_data struct
data DemoData = DemoData { flyState :: Word32
                         , batteryPercentage :: Word32
                         , demoTheta :: Float
                         , demoPhi :: Float
                         , demoPsi :: Float
                         , demoAltitude :: Word32
                         , demoVelocity :: Vector
                         } deriving (Show)

--Navdata package header
data Header = Header { header :: Word32
                     , state :: Word32
                     , sequenceNr :: Word32
                     , visionFlag :: Word32
                     } deriving (Show)

--Datatype holding the drone time
data Time = Time { droneTime :: Word32
                 } deriving (Show)

--Datatype holding the raw meassurements of the drone sensors
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

--Datatype holding the importtant values from the phys_measures struct
--accelerometer in m/s^2
data PhysMeasures = PhysMeasures { accelerometers :: Vector
                                 , gyroscopes :: Vector
                                 } deriving (Show)

--Datatype holding information about the offsets of the gyroscope
data GyroOffsets = GyroOffsets { gOffsets :: Vector } deriving (Show)

--Datatype holding the euler angles theta and phi
data EulerAngles = EulerAngles { eulerTheta :: Float
                               , eulerPhi :: Float
                               } deriving (Show)

--Datatype holding the values of the references struct
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

data Trims = Trims { tAngularRatesR :: Float
                   , tEulerTheta :: Float
                   , tEulerPhi :: Float
                   } deriving (Show)

data RcReferences = RcReferences { rcPitch :: Int
                                 , rcRoll :: Int
                                 , rcYaw :: Int
                                 , rcGaz :: Int
                                 , rcAg :: Int
                                 } deriving (Show)

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

data VisionRaw = VisionRaw { vrTx :: Float
                           , vrTy :: Float
                           , vrTz :: Float
                           } deriving (Show)

data VisionOf = VisionOf { voDx :: (Float, Float, Float, Float, Float)
                         , voDy :: (Float, Float, Float, Float, Float)
                         } deriving (Show)

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

data VisionPerf = VisionPerf { vpSzo :: Float
                             , vpCorners :: Float
                             , vpCompute :: Float
                             , vpTracking :: Float
                             , vpTrans :: Float
                             , vpUpdate :: Float
                             , custom :: [Float]
                             } deriving (Show)

data TrackersSend = TrackersSend { locked :: [Int]
                                 , point :: [(Float, Float)]
                                 } deriving (Show)

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

data Watchdog = Watchdog { wd :: Word32 } deriving (Show)

data AdcDataFrame = AdcDataFrame { adcVersion :: Word32
                                 , adcFrame :: [Word8]
                                 } deriving (Show)

data VideoStream = VideoStream { vsQuant :: Word8
                               , vsFrame :: (Word32, Word32)
                               , vsAtcmd :: (Word32, Word32, Float, Word32)
                               , vsBitrate :: (Word32, Word32)
                               , vsData :: [Int]
                               , tcpQueueLevel :: Word32
                               , fifoQueueLevel :: Word32
                               } deriving (Show)

data Games = Games { counters :: (Word32, Word32) } deriving (Show)

data PressureRaw = PressureRaw { prUp :: Int
                               , prUt :: Word16
                               , prTemperature :: Int16
                               , prPressure :: Int
                               } deriving (Show)

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

data Windspeed = Windspeed { wsSpeed :: Float
                           , wsAngle :: Float
                           , wsCompensation :: (Float, Float)
                           , wsStateX :: [Float]
                           , wsDebug :: [Float]
                           } deriving (Show)

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

data HDVideoStream = HDVideoStream { hdState :: Word32
                                   , hdStorage :: (Word32, Word32)
                                   , hdUsbkey :: (Word32, Word32)
                                   , hdFrameNumber :: Word32
                                   , hdUsbRemainingTime :: Word32
                                   } deriving (Show)

data Wifi = Wifi { wifiLinkQuality :: Float } deriving (Show)

data SatChannel = SatChannel { scSat :: Word8
                             , scCn0 :: Word8
                             } deriving (Show)

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

data CheckSum = CheckSum { value :: Word32 } deriving (Show)

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

emptyNavData :: NavData
emptyNavData = NavData Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing
