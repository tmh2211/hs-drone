{-# LANGUAGE Arrows #-}

import Control.Monad.Trans
import Control.Monad.State
import Data.MonadicStreamFunction
import Data.Maybe
import Data.Monoid
import Data.Matrix

import Control.Monad.Drone
import Robotics.ArDrone.NavDataParser

dt = 0.5 :: Float
x0 = 2 :: Float

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
    matrix <- gets calibrationMatrix
    --takeOff
    reactimate $ proc () -> do
      navData <- liftMStreamF_ getNavData -< ()
      let accVector = vectorToMatrix $ fromMaybe (Vector 0 0 0) $ accelerometers <$> physMeasures navData
      let deltaVector = fromMaybe (Vector 0 0 0) $ viDelta <$> vision navData
      let rotMatrix = createRotationMatrix deltaVector
      let calibAccVector = multStd rotMatrix (multStd matrix accVector)
      let accX = getElem 1 1 calibAccVector
      posX <- integralFrom 0 <<< integralFrom 0 -< accX
      _ <- liftMStreamF $ liftIO . putStrLn -< show posX ++ "\t" ++ show (getElem 1 1 calibAccVector) ++ "\t" ++  show (getElem 2 1 calibAccVector) ++ "\t" ++show (getElem 3 1 calibAccVector)
      if abs posX > x0 then liftMStreamF_ land -< () else do
        liftMStreamF flyForward -< 0.01
        liftMStreamF wait -< dt
  return ()

createRotationMatrix :: Vector -> Matrix Float
createRotationMatrix (Vector phi theta psi) = let z = createAxisRotationMatrix 3 psi;
                                                  x = createAxisRotationMatrix 1 phi;
                                                  y = createAxisRotationMatrix 2 theta
                                              in multStd (multStd z x) y

createAxisRotationMatrix :: Int -> Float -> Matrix Float
createAxisRotationMatrix 1 angle = fromLists [[ 1, 0, 0], [ 0, cos angle, -sin angle], [ 0, sin angle, cos angle]]
createAxisRotationMatrix 2 angle = fromLists [[ cos angle, 0, sin angle], [ 0, 1, 0], [ -sin angle, 0, cos angle]]
createAxisRotationMatrix _ angle = fromLists [[ cos angle, -sin angle, 0], [ sin angle, cos angle, 0], [ 0, 0, 1]]
