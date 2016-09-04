import Control.Monad.Drone
import Robotics.ArDrone.Control hiding (runDrone, main)
import Control.Monad.Trans

main :: IO ()
main = do
  result <- runDrone $ do
    initNavaData
    mainLoop
  case result of
    Left e -> liftIO $ putStrLn $ show e
    Right r -> return ()

mainLoop :: Drone ()
mainLoop = do
  nd <- getNavData
  liftIO $ print nd
  wait 0.01
  mainLoop
