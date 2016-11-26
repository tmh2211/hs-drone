import Robotics.ArDrone.Repl.ReplCommands

welcomeMsg = "Welcome to the interactive drone command center.\n\
             \Now you can control the ArDrone 2.0 from the command line.\n\
             \Currently you have the following commands available:\n\
             \Initialize\t--connects to the drone\n\
             \TakeOff\t--drone will start flying\n\
             \Stop\t--stops the last command from being executed (drone will hover)\n\
             \Land\t--will land the drone on the ground\n\
             \Forward <n>\t--the drone will fly forward: n between 0 (no power) and 1 (max power)\n\
             \Backwards <n>\t--the drone will fly backwards\n\
             \Left <n>\t--the drone will fly to the left\n\
             \Right <n>\t--the drone will fly to the right\n\
             \Up <n>\t--the drone will fly upwards\n\
             \Down <n>\t--the drone will fly downwards\n\
             \Clockwise <n>\t--the drone will circle clockwise\n\
             \CounterClockwise <n>\t--the drone will circle counterclockwise\n\
             \Exit\t--will terminate this progarm\n\
             \Type one of those commands and press enter. Have fun."

main :: IO ()
main = do
  putStrLn welcomeMsg
  mainLoop

mainLoop :: IO ()
mainLoop = do
  line <- getLine
  let cmd = readReplCommand line
  case cmd of
    Just Exit -> putStrLn "Bye."
    Just c -> do
      putStrLn $ "That is a correct command " ++ show c
      mainLoop
    Nothing -> do
      putStrLn "Incorrect command. Drone state remains unchanged."
      mainLoop
