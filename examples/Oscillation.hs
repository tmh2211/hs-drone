{-# LANGUAGE Arrows #-}

import Control.Monad.Trans
import Control.Monad.State
import Data.MonadicStreamFunction
import Data.Maybe
import Data.Monoid
import Data.Matrix

import Control.Monad.Drone
import Robotics.ArDrone.NavDataParser
import Robotics.ArDrone.NavDataConstants

dt = 0.5 :: Float
x0 = 0.05 :: Float

mySumFrom :: (Num a, Monad m) => a -> MStreamF m a a
mySumFrom a = arr Sum >>> sumFrom (Sum a) >>> arr getSum

integralFrom :: (Monad m) => Float -> MStreamF m Float Float
integralFrom a = arr (* dt) >>> mySumFrom a

main :: IO ()
main = do
  runDrone $ do
    disableEmergency
    ftrim
    initNavaData
    configureNavDataOptions [PHYS_MEASURES, VISION]
    matrix <- gets calibrationMatrix
    takeOff
    reactimate $ proc () -> do
      navData <- liftMStreamF_ getNavData -< ()
      let accVector = vectorToMatrix $ fromMaybe (Vector 0 0 0) $ accelerometers <$> physMeasures navData
      let eulerAngles = fromMaybe (Eulers 0 0 0) $ viEulerAngles <$> vision navData
      let rotMatrix = createRotationMatrix eulerAngles
      let calibAccVector = multStd rotMatrix (multStd matrix accVector)
      let accX = getElem 1 1 calibAccVector
      posX <- integralFrom 0 <<< integralFrom 0 -< accX
      --_ <- liftMStreamF $ liftIO . putStrLn -< show eulerAngles
      --_ <- liftMStreamF $ liftIO . putStrLn -< show posX ++ "\t" ++ show (getElem 1 1 calibAccVector) ++ "\t" ++  show (getElem 2 1 calibAccVector) ++ "\t" ++show (getElem 3 1 calibAccVector)
      _ <- liftMStreamF $ liftIO . putStrLn -< show accX
      --_ <- liftMStreamF $ liftIO . putStrLn -< show posX
      if abs posX > x0
        then do
          liftMStreamF flyBackwards -< 0.01
          liftMStreamF wait -< dt
        else do
          liftMStreamF flyForward -< 0.01
          liftMStreamF wait -< dt
  return ()

createRotationMatrix :: Eulers -> Matrix Float
createRotationMatrix (Eulers phi theta psi) = let z = createZAxisRotationMatrix psi;
                                                  x = createXAxisRotationMatrix phi;
                                                  y = createYAxisRotationMatrix theta
                                              in multStd (multStd x y) z

createXAxisRotationMatrix :: Float -> Matrix Float
createXAxisRotationMatrix angle = fromLists [[ 1, 0, 0], [ 0, cos angle, -sin angle], [ 0, sin angle, cos angle]]

createYAxisRotationMatrix :: Float -> Matrix Float
createYAxisRotationMatrix angle = fromLists [[ cos angle, 0, sin angle], [ 0, 1, 0], [ -sin angle, 0, cos angle]]

createZAxisRotationMatrix :: Float -> Matrix Float
createZAxisRotationMatrix angle = fromLists [[ cos angle, -sin angle, 0], [ sin angle, cos angle, 0], [ 0, 0, 1]]
