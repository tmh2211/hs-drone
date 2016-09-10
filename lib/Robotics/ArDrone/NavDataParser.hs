module Robotics.ArDrone.NavDataParser
( parseNavData
, runGet
, BS.fromStrict
, emptyNavData
, vectorToMatrix
, module Robotics.ArDrone.NavDataTypes
) where

import qualified Data.ByteString.Lazy as BS
import Data.Binary.Get
import Data.Word
import Data.Matrix

import Robotics.ArDrone.NavDataTypes

parseNavData :: Get NavData
parseNavData = do
  header <- getHeader
  getNavData (NavData (Just header) Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing)

getNavData :: NavData -> Get NavData
getNavData nd@(NavData h d t r p go ea refs vi c) = do
  id <- getWord16le
  size <- getWord16le
  case id of
    0     -> do demoData <- getDemoData
                getNavData (NavData h (Just demoData) t r p go ea refs vi c)
    1     -> do time <- getTime
                getNavData (NavData h d (Just time) r p go ea refs vi c)
    2     -> do rawMeassures <- getRawMeassures
                getNavData (NavData h d t (Just rawMeassures) p go ea refs vi c)
    3     -> do physMeasures <- getPhysMeasures
                getNavData (NavData h d t r (Just physMeasures) go ea refs vi c)
    4     -> do gyroOffsets <- getGyroOffsets
                getNavData (NavData h d t r p (Just gyroOffsets) ea refs vi c)
    5     -> do eulerAngles <- getEulerAngles
                getNavData (NavData h d t r p go (Just eulerAngles) refs vi c)
    6     -> do references <- getReferences
                getNavData (NavData h d t r p go ea (Just references) vi c)
    13    -> do vision <- getVision
                getNavData (NavData h d t r p go ea refs (Just vision) c)
    65535 -> do cks <- getCheckSum
                return (NavData h d t r p go ea refs vi (Just cks))
    _     -> do skip (fromIntegral size - 4)
                getNavData nd

getVision :: Get Vision
getVision = do
  state <- getWord32le
  misc <- getInt32le
  phiTrim <- getFloatle
  phiProp <- getFloatle
  thetaTrim <- getFloatle
  thetaProp <- getFloatle
  newRawPicture <- getInt32le
  capTheta <- getFloatle
  capPhi <- getFloatle
  capPsi <- getFloatle
  capAlt <- getInt32le
  time <- getWord32le
  bodyVX <- getFloatle
  bodyVY <- getFloatle
  bodyVZ <- getFloatle
  deltaPhi <- getFloatle
  deltaTheta <- getFloatle
  deltaPsi <- getFloatle
  goldDef <- getWord32le
  goldReset <- getWord32le
  goldX <- getFloatle
  goldY <- getFloatle
  return (Vision state (fromIntegral misc) phiTrim phiProp thetaTrim thetaProp (fromIntegral newRawPicture) capTheta capPhi capPsi (fromIntegral capAlt) time (Vector bodyVX bodyVY bodyVZ) (Vector deltaPhi deltaTheta deltaPsi) goldDef goldReset goldX goldY)


getReferences :: Get References
getReferences = do
  theta <- getInt32le
  phi <- getInt32le
  thetaI <- getInt32le
  phiI <- getInt32le
  pitch <- getInt32le
  roll <- getInt32le
  yaw <- getInt32le
  psi <- getInt32le
  vx <- getFloatle
  vy <- getFloatle
  thetaMod <- getFloatle
  phiMod <- getFloatle
  kVX <- getFloatle
  kVY <- getFloatle
  kMode <- getInt32le
  uiTime <- getFloatle
  uiTheta <- getFloatle
  uiPhi <- getFloatle
  uiPsi <- getFloatle
  uiPsiAccuracy <- getFloatle
  uiSeq <- getInt32le
  return (References (fromIntegral theta) (fromIntegral phi) (fromIntegral thetaI) (fromIntegral phiI) (fromIntegral pitch) (fromIntegral roll) (fromIntegral yaw) (fromIntegral psi) vx vy thetaMod phiMod kVX kVY (fromIntegral kMode) uiTime uiTheta uiPhi uiPsi uiPsiAccuracy (fromIntegral uiSeq))

getEulerAngles :: Get EulerAngles
getEulerAngles = do
  theta <- getFloatle
  phi <- getFloatle
  return (EulerAngles theta phi)

getGyroOffsets :: Get GyroOffsets
getGyroOffsets = do
  x <- getFloatle
  y <- getFloatle
  z <- getFloatle
  return (GyroOffsets (Vector x y z))

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
  let xa_ = xa/1000*9.81
  ya <- getFloatle
  let ya_ = ya/1000*9.81
  za <- getFloatle
  let za_ = za/1000*9.81
  xg <- getFloatle
  yg <- getFloatle
  zg <- getFloatle
  _ <- getWord32le  -- drop alim3V3
  _ <- getWord32le  -- drop vrefEpson
  _ <- getWord32le  -- drop vrefIDG
  return (PhysMeasures (Vector xa_ ya_ za_) (Vector xg yg zg))

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
