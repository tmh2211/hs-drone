module Robotics.ArDrone.Repl.ReplServer where

import Robotics.ArDrone.Repl.ReplCommands as RC
import Control.Monad.Drone

import Data.IORef
import Control.Monad.Trans


runServer :: IORef ReplCommand -> IO ()
runServer ref = do
  result <- runDrone WithoutVideo $ mainLoop ref
  return ()

mainLoop :: IORef ReplCommand -> Drone ()
mainLoop ref = do
  wait 0.02
  cmd <- liftIO $ readIORef ref
  executeReplCommand cmd
  mainLoop ref


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
