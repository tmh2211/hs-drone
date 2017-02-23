{-|
Module:         Robotics.ArDrone.Repl.ReplServer
Description:    Server that runs in the background of the REPL and send commands.
License:        GPL-2
Maintainer:     thomasmhergenroeder@gmail.com

This module offers a server that runs in the background and waits for commands
written to an IORef. It then converts those commands to an action and lets the
drone execute the command.
-}
module Robotics.ArDrone.Repl.ReplServer where

import Robotics.ArDrone.Repl.ReplCommands as RC
import Control.Monad.Drone

import Data.IORef
import Control.Monad.Trans

-- | Starts the server
runServer :: IORef ReplCommand -> IO ()
runServer ref = do
  result <- runDrone WithoutVideo $ mainLoop ref
  return ()

-- | This functions waits for new commands. If there a none it keeps the
-- connection alive via the wait function.
mainLoop :: IORef ReplCommand -> Drone ()
mainLoop ref = do
  wait 0.02
  cmd <- liftIO $ readIORef ref
  executeReplCommand cmd
  mainLoop ref

-- | This matches a given REPL command to a message the should be send.
executeReplCommand :: ReplCommand -> Drone ()
executeReplCommand None = return ()
executeReplCommand TakeOff = takeOff
executeReplCommand Land = land
executeReplCommand Stop = stop
executeReplCommand (Forward n) = flyForward n
executeReplCommand (Backwards n) = flyBackwards n
executeReplCommand (RC.Left n) = flyLeft n
executeReplCommand (RC.Right n) = flyRight n
executeReplCommand (RC.Up n) = flyUp n
executeReplCommand (RC.Down n) = flyDown n
executeReplCommand (RC.Clockwise n) = rotateClockwise n
executeReplCommand (RC.CounterClockwise n) = rotateCounterClockwise n
