import Control.Monad.Drone

import Control.Concurrent
import Control.Monad.Trans
import qualified Data.ByteString as BS

main :: IO ()
main = do
  runDrone WithVideo $ do
    initNavaData
    mainLoop
  return ()

mainLoop :: Drone ()
mainLoop = do
  imgIORef <- gets imgBytes
  maybeBytes <- liftIO $ readIORef imgIORef
  case maybeBytes of
    Just bs -> liftIO $ BS.writeFile "imgs/image.png" bs
    Nothing -> return ()
  mainLoop
