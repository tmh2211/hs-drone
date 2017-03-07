import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

import Control.Monad.Drone
import Robotics.ArDrone.NavDataConstants
import Robotics.ArDrone.NavDataParser

main :: IO ()
main = do
  result <- runDrone WithoutVideo $ do
    initNavaData
    ftrim
    configureNavDataOptions [PHYS_MEASURES]
    mainLoop
  case result of
    Left e -> liftIO $ putStrLn $ show e
    Right r -> return ()

mainLoop :: Drone ()
mainLoop = do
  nd <- getNavData
  let opt = trackersSend nd
  case opt of
    Nothing -> return ()
    Just v -> do
      liftIO $ putStrLn $ show v
  --liftIO $ print nd
  wait 0.1
  mainLoop
