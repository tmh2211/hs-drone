module Robotics.ArDrone.NavDataParser
( parseNavData
, runGet
, BS.fromStrict
, NavData(..)
) where

import qualified Data.ByteString.Lazy as BS
import Data.Binary.Get
import Data.Word


data Vector = Vector { x :: Float
                     , y :: Float
                     , z :: Float
                     } deriving (Show)

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
                     , seqNr :: Word32
                     , visionFlag :: Word32
                     } deriving (Show)

data CheckSum = CheckSum { value :: Word32 } deriving (Show)

data NavData = NavData { navDataHeader :: Maybe Header
                       , demoData :: Maybe DemoData
                       , physMeasures :: Maybe PhysMeasures
                       , cks :: Maybe CheckSum
                       } deriving (Show)

parseNavData :: Get NavData
parseNavData = do
  header <- getHeader
  getNavData (NavData (Just header) Nothing Nothing Nothing)

getNavData :: NavData -> Get NavData
getNavData nd@(NavData h d p c) = do
  id <- getWord16le
  size <- getWord16le
  case id of
    0     -> do demoData <- getDemoData
                getNavData (NavData h (Just demoData) p c)
    3     -> do physMeasures <- getPhysMeasures
                getNavData (NavData h d (Just physMeasures) c)
    65535 -> do cks <- getCheckSum
                return (NavData h d p (Just cks))
    _     -> do skip (fromIntegral size - 4)
                getNavData nd


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
