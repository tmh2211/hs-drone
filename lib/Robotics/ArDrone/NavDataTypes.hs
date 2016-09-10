module Robotics.ArDrone.NavDataTypes where

import Data.Matrix
import Data.Word

data Vector = Vector { x :: Float
                     , y :: Float
                     , z :: Float
                     } deriving (Show, Read)

instance Monoid Vector where
  mempty = Vector 0 0 0
  Vector x1 y1 z1 `mappend` Vector x2 y2 z2 = Vector (x1 + x2) (y1 + y2) (z1 + z2)

vectorToMatrix :: Vector -> Matrix Float
vectorToMatrix (Vector x y z) = fromLists [[x],[y],[z]]

--Datatype holding the important data from the the demo_data struct
data DemoData = DemoData { flyState :: Word32
                         , batteryPercentage :: Word32
                         , theta :: Float
                         , phi :: Float
                         , psi :: Float
                         , altitude :: Word32
                         , velocity :: Vector
                         } deriving (Show)


--Datatype holding the importtant values from the phys_measures struct
--accelerometer in m/s^2
data PhysMeasures = PhysMeasures { accelerometers :: Vector
                                 , gyroscopes :: Vector
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
data RawMeassures = RawMeassures { accVector :: (Word16, Word16, Word16)
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

data Vision = Vision { viState :: Word32
                     , viMisc :: Int
                     , viPhiTrim :: Float
                     , viPhiRefProp :: Float
                     , viThetaTrim :: Float
                     , viThetaProp :: Float
                     , newRawPicture :: Int
                     , viCaptureTheta :: Float
                     , viCapturePhi :: Float
                     , viCapturePsi :: Float
                     , viCaptureAltitude :: Int
                     , viCaptureTime :: Word32
                     , viBodyV :: Vector
                     , viDelta :: Vector
                     , viGoldDefined :: Word32
                     , viGoldReset :: Word32
                     , viGoldX :: Float
                     , viGoldY :: Float
                     } deriving (Show)

data CheckSum = CheckSum { value :: Word32 } deriving (Show)

data NavData = NavData { navDataHeader :: Maybe Header
                       , demoData :: Maybe DemoData
                       , time :: Maybe Time
                       , rawMeassures :: Maybe RawMeassures
                       , physMeasures :: Maybe PhysMeasures
                       , gyroOffsets :: Maybe GyroOffsets
                       , eulerAngles :: Maybe EulerAngles
                       , references :: Maybe References
                       , vision :: Maybe Vision
                       , cks :: Maybe CheckSum
                       } deriving (Show)

emptyNavData :: NavData
emptyNavData = NavData Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing
