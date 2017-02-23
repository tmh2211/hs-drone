{-|
Module:           Robotics.ArDrone.VideoStreamServer
Description:      This module handles the reception of the video stream
License:          GPL-2
Maintainer:       thomasmhergenroeder@gmail.com
Stability:        Tested on Windows 10

This module implements the logic for the video stream server, that delivers the
front camera pictures as a stream of .png images. It starts the server from the
node.js library and acts as a client to it, receiving the images. On the other
side it acts as a server to the Haskell application and delivers the images via
an IORef.

node.js implementation taken from: https://github.com/felixge/node-ar-drone
-}
module Robotics.ArDrone.VideoStreamServer where
import Control.Concurrent
import Control.Exception
import Network.Socket
import qualified Network.Socket.ByteString as NBS
import qualified Data.ByteString as BS
import qualified Data.ByteString.Char8 as BSC
import Data.IORef
import Data.Binary.Get
import System.Command
import Codec.Picture

-- | This function starts the node.js server as new process. It then starts a
-- TCP server on port 11111 where it waits for the image stream.
runVideoServer :: IORef (Maybe BS.ByteString) -> IO ()
runVideoServer lastFrame = do
  path <- readNodejsPath "nodejs.config"
  putStrLn path
  sock <- socket AF_INET Stream 0
  setSocketOption sock ReuseAddr 1
  bind sock (SockAddrInet 11111 iNADDR_ANY)
  listen sock 2
  createProcess $ shell $ "node " ++ path
  mainLoop sock lastFrame

-- | This function simply accepts connections to the server socket and handles
-- them in different threads in case more than one process connects to it, so it
-- isn't blocking. However, one should never connect more than one process to
-- this socket.
mainLoop :: Socket -> IORef (Maybe BS.ByteString) -> IO ()
mainLoop sock lastFrame = do
  conn <- accept sock
  forkIO $ runConn conn lastFrame
  mainLoop sock lastFrame

-- | This function handles a single connection to the server. It receives the
-- bytes from the node.js server and writes them as a bytestring to an IORef so
-- the Haskell application can access it.
runConn :: (Socket, SockAddr) -> IORef (Maybe BS.ByteString)-> IO ()
runConn conn@(sock, _) lastFrame = do
  msg <- NBS.recv sock 1000000
  writeIORef lastFrame $ Just msg
  runConn conn lastFrame

-- | This function checks if the nodejs.config file exists. This should contain
-- the path to the node.js server script that sends the images to port 11111.
-- If the path is not configured or not configured correctly the application
-- will exit with failure, as it can not recover from that.
-- Should be improved in future versions.
readNodejsPath :: String -> IO (String)
readNodejsPath config = do
  bytes <- (try :: IO BS.ByteString -> IO (Either IOException BS.ByteString)) $ BS.readFile config
  case bytes of
    Left e -> do
      putStrLn "You didn't create a nodejs.config file. Create it and put the location of the repl.js file in there."
      exitFailure
    Right b -> do
      let string = BSC.unpack b
      return string
