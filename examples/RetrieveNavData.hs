import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

import Control.Monad.Drone
import Robotics.ArDrone.NavDataConstants
import Robotics.ArDrone.NavDataParser

main :: IO ()
main = do
  result <- runDrone $ do
    initNavaData
    ftrim
    configureNavDataOptions [VISION]
    mainLoop
  case result of
    Left e -> liftIO $ putStrLn $ show e
    Right r -> return ()

mainLoop :: Drone ()
mainLoop = do
  nd <- getNavData
  let opt = vision nd
  case opt of
    Nothing -> return ()
    Just v -> do
      let euler = viBodyV v
      liftIO $ putStrLn $ show euler
  --liftIO $ print nd
  wait 0.1
  mainLoop
