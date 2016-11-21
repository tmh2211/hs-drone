module Robotics.ArDrone.NavDataParser
( parseNavData
, runGet
, BS.fromStrict
, module Robotics.ArDrone.NavDataTypes
) where

import qualified Data.ByteString.Lazy as BS
import Data.Binary.Get
import Data.Word
import Data.Matrix
import Control.Monad

import Robotics.ArDrone.NavDataTypes

parseNavData :: Get NavData
parseNavData = do
  header <- getHeader
  let empty = emptyNavData
  getNavData (empty { navDataHeader = Just header})

getNavData :: NavData -> Get NavData
getNavData nd = do
  id <- getWord16le
  size <- getWord16le
  case id of
    0     -> do dd <- getDemoData
                getNavData (nd { demoData = Just dd })
    1     -> do t <- getTime
                getNavData (nd { time = Just t })
    2     -> do rm <- getRawMeassures
                getNavData (nd { rawMeasures = Just rm })
    3     -> do pm <- getPhysMeasures
                getNavData (nd { physMeasures = Just pm })
    4     -> do go <- getGyroOffsets
                getNavData (nd { gyroOffsets = Just go })
    5     -> do ea <- getEulerAngles
                getNavData (nd { eulerAngles = Just ea })
    6     -> do refs <- getReferences
                getNavData (nd { references = Just refs })
    7     -> do ts <- getTrims
                getNavData (nd { trims = Just ts })
    8     -> do rcrefs <- getRcReferences
                getNavData (nd { rcReferences = Just rcrefs })
    9     -> do pwm_ <- getPwm
                getNavData (nd { pwm = Just pwm_ })
    10    -> do alt <- getAltitude
                getNavData (nd { altitude = Just alt })
    11    -> do vr <- getVisionRaw
                getNavData (nd { visionRaw = Just vr })
    12    -> do vo <- getVisionOf
                getNavData (nd { visionOf = Just vo })
    13    -> do vis <- getVision
                getNavData (nd { vision = Just vis })
    14    -> do vperf <- getVisionPerf
                getNavData (nd { visionPerf = Just vperf })
    15    -> do tSend <- getTrackersSend
                getNavData (nd { trackersSend = Just tSend })
    16    -> do vd <- getVisionDetect
                getNavData (nd { visionDetect = Just vd })
    17    -> do wd <- getWatchdog
                getNavData (nd { watchdog = Just wd })
    18    -> do adc <- getAdcDataFrame
                getNavData (nd { adcDataFrame = Just adc })
    19    -> do vidstr <- getVideoStream
                getNavData (nd { videoStream = Just vidstr })
    20    -> do g <- getGames
                getNavData (nd { games = Just g })
    21    -> do pr <- getPressureRaw
                getNavData (nd { pressureRaw = Just pr })
    22    -> do mag <- getMagneto
                getNavData (nd { magneto = Just mag })
    23    -> do ws <- getWindspeed
                getNavData (nd { windspeed = Just ws })
    24    -> do kp <- getKalmanPressure
                getNavData (nd { kalmanPressure = Just kp })
    25    -> do hd <- getHDVideoStream
                getNavData (nd { hdVideoStream = Just hd })
    26    -> do wifi_ <- getWifi
                getNavData (nd { wifi = Just wifi_ })
    27    -> do gps_ <- getGps
                getNavData (nd { gps = Just gps_ })
    65535 -> do cks <- getCheckSum
                return (nd { checkSum = Just cks })
    _     -> do skip (fromIntegral size - 4)
                getNavData nd

getTuple :: Get a -> Get (a, a)
getTuple g = do
  first <- g
  second <- g
  return (first, second)

getTriple :: Get a -> Get (a, a, a)
getTriple g = do
  first <- g
  second <- g
  third <- g
  return (first, second, third)

getQuadrouple :: Get a -> Get (a, a, a, a)
getQuadrouple g = do
  first <- g
  second <- g
  third <- g
  fourth <- g
  return (first, second, third, fourth)

getQuintuple :: Get a -> Get (a, a, a, a, a)
getQuintuple g = do
  first <- g
  second <- g
  third <- g
  fourth <- g
  fifth <- g
  return (first, second, third, fourth, fifth)

get3x3Matrix :: Get (Matrix Float)
get3x3Matrix = do
  row1 <- replicateM 3 getFloatle
  row2 <- replicateM 3 getFloatle
  row3 <- replicateM 3 getFloatle
  return $ fromLists [row1, row2, row3]

getVector3x1 :: Get Vector
getVector3x1 = do
  x <- getFloatle
  y <- getFloatle
  z <- getFloatle
  return $ Vector x y z

getSatChannel :: Get SatChannel
getSatChannel = do
  sat <- getWord8
  cn0 <- getWord8
  return $ SatChannel sat cn0

getGps :: Get Gps
getGps = do
  lat <- getDoublele
  lon <- getDoublele
  ele <- getDoublele
  hdop <- getDoublele
  dataAv <- getInt
  zeroVal <- getInt
  wptVal <- getInt
  lat0_ <- getDoublele
  lon0_ <- getDoublele
  latFuse_ <- getDoublele
  lonFuse_ <- getDoublele
  state <- getWord32le
  xtraj <- getFloatle
  xref <- getFloatle
  ytraj <- getFloatle
  yref <- getFloatle
  thetap <- getFloatle
  phip <- getFloatle
  thetai <- getFloatle
  phii <- getFloatle
  thetad <- getFloatle
  phid <- getFloatle
  vd <- getDoublele
  pd <- getDoublele
  gpsspeed <- getFloatle
  lastframe <- getWord32le
  gpsdegree <- getFloatle
  degreeMag <- getFloatle
  gpsephe <- getFloatle
  gpsehve <- getFloatle
  cn0 <- getFloatle
  sats <- getWord32le
  channels <- replicateM 12 getSatChannel
  plugged <- getInt
  epheStatus <- getWord32le
  vxtraj <- getFloatle
  vytraj <- getFloatle
  firmStatus <- getWord32le
  return $ Gps lat lon ele hdop dataAv zeroVal wptVal lat0_ lon0_ latFuse_ lonFuse_ state xtraj xref ytraj yref thetap phip thetai phii thetad phid vd pd gpsspeed lastframe gpsdegree degreeMag gpsephe gpsehve cn0 sats channels plugged epheStatus vxtraj vytraj firmStatus

getWifi :: Get Wifi
getWifi = do
  quality <- getFloatle
  return $ Wifi quality

getHDVideoStream :: Get  HDVideoStream
getHDVideoStream = do
  hdState <- getWord32le
  hdStorage <- getTuple getWord32le
  usbkey <- getTuple getWord32le
  frameN <- getWord32le
  remainingTime <- getWord32le
  return $ HDVideoStream hdState hdStorage usbkey frameN remainingTime

getKalmanPressure :: Get KalmanPressure
getKalmanPressure = do
  offset <- getFloatle
  kpalt <- getFloatle
  kpvel <- getFloatle
  kpangle <- getTuple getFloatle
  kpus <- getTuple getFloatle
  kpcov <- getTriple getFloatle
  groundEffect <- getWord8
  kpsum <- getFloatle
  kpreject <- getWord8
  umultisinus <- getFloatle
  gazalt <- getFloatle
  flagMultisinus <- getWord8
  flagMultisinusStart <- getWord8
  return $ KalmanPressure offset kpalt kpvel kpangle kpus kpcov groundEffect kpsum kpreject umultisinus gazalt flagMultisinus flagMultisinusStart

getWindspeed :: Get Windspeed
getWindspeed = do
  speed <- getFloatle
  angle <- getFloatle
  compensation <- getTuple getFloatle
  stateX <- replicateM 6 getFloatle
  debug <- replicateM 3 getFloatle
  return $ Windspeed speed angle compensation stateX debug

getMagneto :: Get Magneto
getMagneto = do
  mx <- getInt16le
  my <- getInt16le
  mz <- getInt16le
  raw <- getVector3x1
  rectified <- getVector3x1
  offset <- getVector3x1
  heading <- getTriple getFloatle
  ok <- getWord8
  state <- getWord32le
  radius <- getFloatle
  err <- getTuple getFloatle
  return $ Magneto mx my mz raw rectified offset heading ok state radius err

getPressureRaw :: Get PressureRaw
getPressureRaw = do
  up <- getInt
  ut <- getWord16le
  temp <- getInt16le
  press <- getInt
  return $ PressureRaw up ut temp press

getGames :: Get Games
getGames = do
  c <- getTuple getWord32le
  return $ Games c

getVideoStream :: Get VideoStream
getVideoStream = do
  quant <- getWord8
  frame <- getTuple getWord32le
  at1 <- getWord32le
  at2 <- getWord32le
  at3 <- getFloatle
  at4 <- getWord32le
  let atcmd = (at1, at2, at3, at4)
  bitrate <- getTuple getWord32le
  d <- replicateM 5 getInt
  tcp <- getWord32le
  fifo <- getWord32le
  return $ VideoStream quant frame atcmd bitrate d tcp fifo

getAdcDataFrame :: Get AdcDataFrame
getAdcDataFrame = do
  version <- getWord32le
  frame <- replicateM 32 getWord8
  return $ AdcDataFrame version frame

getWatchdog :: Get Watchdog
getWatchdog = do
  wd <- getWord32le
  return $ Watchdog wd

getVisionDetect :: Get VisionDetect
getVisionDetect = do
  nbd <- getWord32le
  t <- replicateM 4 getWord32le
  xc <- replicateM 4 getWord32le
  yc <- replicateM 4 getWord32le
  width <- replicateM 4 getWord32le
  height <- replicateM 4 getWord32le
  dist <- replicateM 4 getWord32le
  orientationAngle <- replicateM 4 getFloatle
  rotation <- replicateM 4 get3x3Matrix
  translation <- replicateM 4 getVector3x1
  cameraSource <- replicateM 4 getWord32le
  return $ VisionDetect nbd t xc yc width height dist orientationAngle rotation translation cameraSource

getTrackersSend :: Get TrackersSend
getTrackersSend = do
  locked <- replicateM 30 getInt
  point <- replicateM 30 $ do
    x <- getFloatle
    y <- getFloatle
    return (x, y)
  return $ TrackersSend locked point

getVisionPerf :: Get VisionPerf
getVisionPerf = do
  szo <- getFloatle
  corners <- getFloatle
  compute <- getFloatle
  tracking <- getFloatle
  trans <- getFloatle
  update <- getFloatle
  skip 80
  return $ VisionPerf szo corners compute tracking trans update []

getVisionOf :: Get VisionOf
getVisionOf = do
  voDx1 <- getFloatle
  voDx2 <- getFloatle
  voDx3 <- getFloatle
  voDx4 <- getFloatle
  voDx5 <- getFloatle
  let voDx = (voDx1, voDx2, voDx3, voDx4, voDx5)
  voDy1 <- getFloatle
  voDy2 <- getFloatle
  voDy3 <- getFloatle
  voDy4 <- getFloatle
  voDy5 <- getFloatle
  let voDy = (voDy1, voDy2, voDy3, voDy4, voDy5)
  return $ VisionOf voDx voDy

getVisionRaw :: Get VisionRaw
getVisionRaw = do
  tx <- getFloatle
  ty <- getFloatle
  tz <- getFloatle
  return $ VisionRaw tx ty tz

getAltitude :: Get Altitude
getAltitude = do
  vision <- getInt
  vel <- getFloatle
  ref <- getInt
  raw <- getInt
  obsAcc <- getFloatle
  obsAlt <- getFloatle
  obsXx <- getFloatle
  obsXy <- getFloatle
  obsXz <- getFloatle
  let obsX = Vector obsXx obsXy obsXz
  obsState <- getWord32le
  estVb1 <- getFloatle
  estVb2 <- getFloatle
  let estVb = (estVb1, estVb2)
  estState <- getWord32le
  return $ Altitude vision vel ref raw obsAcc obsAlt obsX obsState estVb estState

--TODO: Not working find out why and fix it
getPwm :: Get Pwm
getPwm = do
  motors <- getWord32le
  satMotors <- getWord32le
  gazFeedForward <- getFloatle
  gazAltitude <- getFloatle
  altitudeIntegral <- getFloatle
  vzRef <- getFloatle
  upitch <- getInt
  uroll <- getInt
  uyaw <- getInt
  yawUI <- getFloatle
  uPitchPlanif <- getInt
  uRollPlanif <- getInt
  uYawPlanif <- getInt
  uGazPlanif <- getFloatle
  motorCurrents1 <- getWord16le
  motorCurrents2 <- getWord16le
  motorCurrents3 <- getWord16le
  motorCurrents4 <- getWord16le
  altitudeProp <- getFloatle
  altitudeDer <- getFloatle
  return (Pwm motors satMotors gazFeedForward gazAltitude altitudeIntegral vzRef upitch uroll uyaw yawUI uPitchPlanif uRollPlanif uYawPlanif uGazPlanif (motorCurrents1, motorCurrents2, motorCurrents3, motorCurrents4) altitudeProp altitudeDer)

getRcReferences :: Get RcReferences
getRcReferences = do
  pitch <- getInt32le
  roll <- getInt32le
  yaw <- getInt32le
  gaz <- getInt32le
  ag <- getInt32le
  return (RcReferences (fromIntegral pitch) (fromIntegral roll) (fromIntegral yaw) (fromIntegral gaz) (fromIntegral ag))

getTrims :: Get Trims
getTrims = do
  angularRate <- getFloatle
  theta <- getFloatle
  phi <- getFloatle
  return (Trims angularRate theta phi)

getVision :: Get Vision
getVision = do
  state <- getWord32le
  misc <- getInt32le
  phiTrim <- getFloatle
  phiProp <- getFloatle
  thetaTrim <- getFloatle
  thetaProp <- getFloatle
  newRawPicture <- getInt
  capTheta <- getFloatle
  capPhi <- getFloatle
  capPsi <- getFloatle
  let euler = Eulers capPhi capTheta capPsi
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
  return (Vision state (fromIntegral misc) phiTrim phiProp thetaTrim thetaProp newRawPicture euler (fromIntegral capAlt) time (Vector bodyVX bodyVY bodyVZ) (Vector deltaPhi deltaTheta deltaPsi) goldDef goldReset goldX goldY)


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
  v <- getVector3x1
  return (GyroOffsets v)

getRawMeassures :: Get RawMeasures
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
  return (RawMeasures (accX, accY, accZ) (gyrX, gyrY, gyrZ) (gyr110X, gyr110Y) batteryMilliVolt usEchoStart usEchoEnd usEchoAssociation usEchoDistance usCurveTime usCurveValue usCurveRef echoFlagIni echoNum echoSum altTemp gradient)

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

getInt :: Get Int
getInt = do
  int <- getInt32le
  return $ fromIntegral int
