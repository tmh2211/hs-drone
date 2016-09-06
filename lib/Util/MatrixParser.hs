module Util.MatrixParser where

import Data.Matrix
--import System.Directory

import Robotics.ArDrone.NavDataParser

data Matrix3x3 = Matrix3x3 { col1 :: Vector
                           , col2 :: Vector
                           , col3 :: Vector
                           } deriving (Show, Read)

writeMatrixToFile :: String -> Matrix Float -> IO ()
writeMatrixToFile path matrix = do
  let m3x3 = fromMatrix matrix
  writeFile path $ show m3x3

readMatrixFromFile :: String -> IO (Matrix Float)
readMatrixFromFile path = do
  contents <- readFile path
  let matrix = toMatrix $ read contents
  return matrix
  --exists <- doesFileExist path
  --case exists of
  --  True -> do contents <- readFile path
  --             let matrix = read contents
  --             return matrix
  --  False -> do putStrLn "The calibration file does not exist. You should create one."
  --              let matrix = scaleMatrix 1000 $ identity 3
  --              return matrix

fromMatrix :: Matrix Float -> Matrix3x3
fromMatrix m = Matrix3x3 (Vector (getElem 1 1 m) (getElem 2 1 m) (getElem 3 1 m)) (Vector (getElem 1 2 m) (getElem 2 2 m) (getElem 3 2 m)) (Vector (getElem 1 3 m) (getElem 2 3 m) (getElem 3 3 m))

toMatrix :: Matrix3x3 -> Matrix Float
toMatrix m = fromLists $ toRowVectorLists m

toRowVectorLists :: Matrix3x3 -> [[Float]]
toRowVectorLists m = [[x $ col1 m, x $ col2 m, x $ col3 m],[y $ col1 m, y $ col2 m, y $ col3 m],[z $ col1 m, z $ col2 m, z $ col3 m]]
