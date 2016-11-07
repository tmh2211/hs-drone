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

import Robotics.ArDrone.PaVEParser

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

mainLoop :: Socket -> IORef (Maybe BS.ByteString) -> IO ()
mainLoop sock lastFrame = do
  conn <- accept sock
  forkIO $ runConn conn lastFrame
  mainLoop sock lastFrame

runConn :: (Socket, SockAddr) -> IORef (Maybe BS.ByteString)-> IO ()
runConn conn@(sock, _) lastFrame = do
  msg <- NBS.recv sock 1000000
  writeIORef lastFrame $ Just msg
  runConn conn lastFrame

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
