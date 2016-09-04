module Robotics.ArDrone.NavDataParser
( parseNavData
, runGet
, BS.fromStrict
, NavData(..)
, PhysMeasures(..)
, Vector(..)
, Header(..)
) where

import qualified Data.ByteString.Lazy as BS
import Data.Binary.Get
import Data.Word


data Vector = Vector { x :: Float
                     , y :: Float
                     , z :: Float
                     } deriving (Show)

instance Monoid Vector where
  mempty = Vector 0 0 0
  Vector x1 y1 z1 `mappend` Vector x2 y2 z2 = Vector (x1 + x2) (y1 + y2) (z1 + z2)


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

data CheckSum = CheckSum { value :: Word32 } deriving (Show)

data NavData = NavData { navDataHeader :: Maybe Header
                       , demoData :: Maybe DemoData
                       , time :: Maybe Time
                       , rawMeassures :: Maybe RawMeassures
                       , physMeasures :: Maybe PhysMeasures
                       , cks :: Maybe CheckSum
                       } deriving (Show)

parseNavData :: Get NavData
parseNavData = do
  header <- getHeader
  getNavData (NavData (Just header) Nothing Nothing Nothing Nothing Nothing)

getNavData :: NavData -> Get NavData
getNavData nd@(NavData h d t r p c) = do
  id <- getWord16le
  size <- getWord16le
  case id of
    0     -> do demoData <- getDemoData
                getNavData (NavData h (Just demoData) t r p c)
    1     -> do time <- getTime
                getNavData (NavData h d (Just time) r p c)
    2     -> do rawMeassures <- getRawMeassures
                getNavData (NavData h d t (Just rawMeassures) p c)
    3     -> do physMeasures <- getPhysMeasures
                getNavData (NavData h d t r (Just physMeasures) c)
    65535 -> do cks <- getCheckSum
                return (NavData h d t r p (Just cks))
    _     -> do skip (fromIntegral size - 4)
                getNavData nd

getRawMeassures :: Get RawMeassures
getRawMeassures = do
  accX <- getWord16le
  accY <- getWord16le
  accZ <- getWord16le
  gyrX <- getWord16le
  gyrY <- getWord16le
  gyrZ <- getWord16le
  gyr110X <- getWord16le
  gyr110Y <- getWord16le
  batteryMilliVolt <- getWord32le
  usEchoStart <- getWord16le
  usEchoEnd <- getWord16le
  usEchoAssociation <- getWord16le
  usEchoDistance <- getWord16le
  usCurveTime <- getWord16le
  usCurveValue <- getWord16le
  usCurveRef <- getWord16le
  echoFlagIni <- getWord16le
  echoNum <- getWord16le
  echoSum <- getWord32le
  altTemp <- getWord32le
  gradient <- getWord16le
  return (RawMeassures (accX, accY, accZ) (gyrX, gyrY, gyrZ) (gyr110X, gyr110Y) batteryMilliVolt usEchoStart usEchoEnd usEchoAssociation usEchoDistance usCurveTime usCurveValue usCurveRef echoFlagIni echoNum echoSum altTemp gradient)

getTime :: Get Time
getTime = do
  time <- getWord32le
  return (Time time)

getHeader :: Get Header
getHeader = do
  header <- getWord32le
  state <- getWord32le
  seqNr <- getWord32le
  vision <- getWord32le
  return (Header header state seqNr vision)

getPhysMeasures :: Get PhysMeasures
getPhysMeasures = do
  _ <- getInt32le   -- drop accelerometer temperature
  _ <- getInt16le   -- drop gyroscope temperature
  xa <- getFloatle
  ya <- getFloatle
  za <- getFloatle
  xg <- getFloatle
  yg <- getFloatle
  zg <- getFloatle
  _ <- getWord32le  -- drop alim3V3
  _ <- getWord32le  -- drop vrefEpson
  _ <- getWord32le  -- drop vrefIDG
  return (PhysMeasures (Vector xa ya za) (Vector xg yg zg))

getDemoData :: Get DemoData
getDemoData = do
  flyState <- getWord32le
  batteryPercentage <- getWord32le
  theta <- getFloatle
  phi <- getFloatle
  psi <- getFloatle
  alt <- getWord32le
  vx <- getFloatle
  vy <- getFloatle
  vz <- getFloatle
  _ <- getWord32le -- drop the frame index of the package
  _ <- getWord32le -- drop the camera rotation matrx 9 x Word32
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le -- drop the camera translation vector 3 x Word32
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le -- drop camera tagindex
  _ <- getWord32le -- drop camera type
  _ <- getWord32le -- drop drone camera rotation matrix 9 x Word32
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le
  _ <- getWord32le -- drop drone camera translation vector 3xWord32
  _ <- getWord32le
  _ <- getWord32le
  return (DemoData flyState batteryPercentage theta phi psi alt (Vector vx vy vz))

getCheckSum :: Get CheckSum
getCheckSum = do
  value <- getWord32le
  return (CheckSum value)
