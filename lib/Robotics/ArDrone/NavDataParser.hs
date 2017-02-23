{-|
Module:       Robotics.ArDrone.NavDataParser
Description:  Functionality to parse the bytes of the navigational data
License:      GPL-2
Maintainer:   thomasmhergenroeder@gmail.com

This module offers the functionality to parse the byte string that is sent from
the navdata port. It has a toplevel function that kicks off the parsing of a
packet, a function that handles the options and one parsing function for each
option. Additionaly there are some helper functions that make the code more
readable.<br>
Information mainly taken from https://github.com/felixge/node-ar-drone.
-}
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

-- | Top level function that kicks of the parsing process. First it parses the
-- header and then the options.
parseNavData :: Get NavData
parseNavData = do
  header <- getHeader
  let empty = emptyNavData
  getNavData (empty { navDataHeader = Just header})

-- | This function parses the navdata options that are present.
getNavData :: NavData -> Get NavData
getNavData nd = do
  -- identify the option by the id
  id <- getWord16le
  size <- getWord16le
  -- decide based on the id of the option which function to call
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

-- | Helper function to parse a tuple
getTuple :: Get a -> Get (a, a)
getTuple g = do
  first <- g
  second <- g
  return (first, second)

  -- | Helper function to parse a triple
getTriple :: Get a -> Get (a, a, a)
getTriple g = do
  first <- g
  second <- g
  third <- g
  return (first, second, third)

  -- | Helper function to parse a quadrouple
getQuadrouple :: Get a -> Get (a, a, a, a)
getQuadrouple g = do
  first <- g
  second <- g
  third <- g
  fourth <- g
  return (first, second, third, fourth)

  -- | Helper function to parse a quintuple
getQuintuple :: Get a -> Get (a, a, a, a, a)
getQuintuple g = do
  first <- g
  second <- g
  third <- g
  fourth <- g
  fifth <- g
  return (first, second, third, fourth, fifth)

  -- | Helper function to parse a 3x3 matrix
get3x3Matrix :: Get (Matrix Float)
get3x3Matrix = do
  row1 <- replicateM 3 getFloatle
  row2 <- replicateM 3 getFloatle
  row3 <- replicateM 3 getFloatle
  return $ fromLists [row1, row2, row3]

  -- | Helper function to parse a vector
getVector3x1 :: Get Vector
getVector3x1 = do
  x <- getFloatle
  y <- getFloatle
  z <- getFloatle
  return $ Vector x y z

-- | Helper function to parse the subtype of a SatChannel
getSatChannel :: Get SatChannel
getSatChannel = do
  sat <- getWord8
  cn0 <- getWord8
  return $ SatChannel sat cn0

-- | Parsing the GPS data
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

-- | Parsing the Wifi option.
getWifi :: Get Wifi
getWifi = do
  quality <- getFloatle
  return $ Wifi quality

-- | Parsing the HDVideoStream option.
getHDVideoStream :: Get  HDVideoStream
getHDVideoStream = do
  hdState <- getWord32le
  hdStorage <- getTuple getWord32le
  usbkey <- getTuple getWord32le
  frameN <- getWord32le
  remainingTime <- getWord32le
  return $ HDVideoStream hdState hdStorage usbkey frameN remainingTime

-- | Parsing the KalmanPressure option.
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

-- | Parsing the Windspeed option.
getWindspeed :: Get Windspeed
getWindspeed = do
  speed <- getFloatle
  angle <- getFloatle
  compensation <- getTuple getFloatle
  stateX <- replicateM 6 getFloatle
  debug <- replicateM 3 getFloatle
  return $ Windspeed speed angle compensation stateX debug

-- | Parsing the magnetometer data.
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


-- | Parsing the raw data from pressure measurement.
getPressureRaw :: Get PressureRaw
getPressureRaw = do
  up <- getInt
  ut <- getWord16le
  temp <- getInt16le
  press <- getInt
  return $ PressureRaw up ut temp press

-- | Parsing the Games option.
getGames :: Get Games
getGames = do
  c <- getTuple getWord32le
  return $ Games c

-- | Parsing the VideoStream option.
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

-- | Parsing the ADCDataFrame option.
getAdcDataFrame :: Get AdcDataFrame
getAdcDataFrame = do
  version <- getWord32le
  frame <- replicateM 32 getWord8
  return $ AdcDataFrame version frame

-- | Parsing the Watchdog option.
getWatchdog :: Get Watchdog
getWatchdog = do
  wd <- getWord32le
  return $ Watchdog wd

-- | Parsing the VisionDetect option.
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

-- | Parsing the TrackersSend option.
getTrackersSend :: Get TrackersSend
getTrackersSend = do
  locked <- replicateM 30 getInt
  point <- replicateM 30 $ getTuple getFloatle
  return $ TrackersSend locked point

-- | Parsing the VisionPerf option.
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

-- | Parsing the VisionOf option.
getVisionOf :: Get VisionOf
getVisionOf = do
  voDx <- getQuintuple getFloatle
  voDy <- getQuintuple getFloatle
  return $ VisionOf voDx voDy

-- | Parsing the VisionRaw option.
getVisionRaw :: Get VisionRaw
getVisionRaw = do
  tx <- getFloatle
  ty <- getFloatle
  tz <- getFloatle
  return $ VisionRaw tx ty tz

-- | Parsing the Altitude option.
getAltitude :: Get Altitude
getAltitude = do
  vision <- getInt
  vel <- getFloatle
  ref <- getInt
  raw <- getInt
  obsAcc <- getFloatle
  obsAlt <- getFloatle
  obsX <- getVector3x1
  obsState <- getWord32le
  estVb <- getTuple getFloatle
  estState <- getWord32le
  return $ Altitude vision vel ref raw obsAcc obsAlt obsX obsState estVb estState

-- | Parsing the PWM option.<br>
-- Do not use it: Still has som weird bug to be fixed in fute versions.
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
  return $ Pwm motors satMotors gazFeedForward gazAltitude altitudeIntegral vzRef upitch uroll uyaw yawUI uPitchPlanif uRollPlanif uYawPlanif uGazPlanif (motorCurrents1, motorCurrents2, motorCurrents3, motorCurrents4) altitudeProp altitudeDer

-- | Parsing the RcReferences option.
getRcReferences :: Get RcReferences
getRcReferences = do
  pitch <- getInt
  roll <- getInt
  yaw <- getInt
  gaz <- getInt
  ag <- getInt
  return $ RcReferences pitch roll yaw gaz ag

-- | Parsing the Trims option.
getTrims :: Get Trims
getTrims = do
  angularRate <- getFloatle
  theta <- getFloatle
  phi <- getFloatle
  return $ Trims angularRate theta phi

-- | Parsing the Vision option.
getVision :: Get Vision
getVision = do
  state <- getWord32le
  misc <- getInt
  phiTrim <- getFloatle
  phiProp <- getFloatle
  thetaTrim <- getFloatle
  thetaProp <- getFloatle
  newRawPicture <- getInt
  capTheta <- getFloatle
  capPhi <- getFloatle
  capPsi <- getFloatle
  let euler = Eulers capPhi capTheta capPsi
  capAlt <- getInt
  time <- getWord32le
  bodyV <- getVector3x1
  deltas <- getVector3x1
  goldDef <- getWord32le
  goldReset <- getWord32le
  goldX <- getFloatle
  goldY <- getFloatle
  return $ Vision state misc phiTrim phiProp thetaTrim thetaProp newRawPicture euler capAlt time bodyV deltas goldDef goldReset goldX goldY

-- | Parsing the References option.
getReferences :: Get References
getReferences = do
  theta <- getInt
  phi <- getInt
  thetaI <- getInt
  phiI <- getInt
  pitch <- getInt
  roll <- getInt
  yaw <- getInt
  psi <- getInt
  vx <- getFloatle
  vy <- getFloatle
  thetaMod <- getFloatle
  phiMod <- getFloatle
  kVX <- getFloatle
  kVY <- getFloatle
  kMode <- getInt
  uiTime <- getFloatle
  uiTheta <- getFloatle
  uiPhi <- getFloatle
  uiPsi <- getFloatle
  uiPsiAccuracy <- getFloatle
  uiSeq <- getInt
  return $ References theta phi thetaI phiI pitch roll yaw psi vx vy thetaMod phiMod kVX kVY kMode uiTime uiTheta uiPhi uiPsi uiPsiAccuracy uiSeq

-- | Parsing the EulerAngles option.
getEulerAngles :: Get EulerAngles
getEulerAngles = do
  theta <- getFloatle
  phi <- getFloatle
  return $ EulerAngles theta phi

-- | Parsing the Gyrooffsets option.
getGyroOffsets :: Get GyroOffsets
getGyroOffsets = do
  v <- getVector3x1
  return $ GyroOffsets v

-- | Parsing the RawMeasures option.
getRawMeassures :: Get RawMeasures
getRawMeassures = do
  acc <- getTriple getWord16le
  gyr <- getTriple getWord16le
  gyr110 <- getTuple getWord16le
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
  return $ RawMeasures acc gyr gyr110 batteryMilliVolt usEchoStart usEchoEnd usEchoAssociation usEchoDistance usCurveTime usCurveValue usCurveRef echoFlagIni echoNum echoSum altTemp gradient

-- | Parsing the drone time in milliseconds.
getTime :: Get Time
getTime = do
  time <- getWord32le
  return $ Time time

-- | Parsing the header of the whole navdata packet.
getHeader :: Get Header
getHeader = do
  header <- getWord32le
  state <- getWord32le
  seqNr <- getWord32le
  vision <- getWord32le
  return $ Header header state seqNr vision

-- | Parsing the PhysMeasures option and convert the accelerometer data to
-- m/s^2.
getPhysMeasures :: Get PhysMeasures
getPhysMeasures = do
  skip 6
  a <- getVector3x1
  g <- getVector3x1
  skip 12
  return $ PhysMeasures (scaleVector (9.81/1000) a) g

-- | Parsing the demo data. Dropping deprecated data.
getDemoData :: Get DemoData
getDemoData = do
  flyState <- getWord32le
  batteryPercentage <- getWord32le
  theta <- getFloatle
  phi <- getFloatle
  psi <- getFloatle
  alt <- getWord32le
  v <- getVector3x1
  skip 108
  return $ DemoData flyState batteryPercentage theta phi psi alt v

-- | Parsing the checksum of the packet.
getCheckSum :: Get CheckSum
getCheckSum = do
  value <- getWord32le
  return $ CheckSum value

-- | Helper function to read an Int.
getInt :: Get Int
getInt = do
  int <- getInt32le
  return $ fromIntegral int
