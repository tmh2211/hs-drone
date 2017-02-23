{-|
Module:             Util.MatrixParser
Description:        Utility module to parse the calibration matrix from the file.
License:            GPL-2
Maintainer:         thomasmaxhergenroeder@gmail.com

This module offers function to parse the calibration matrix from a file and also
write it to a file. To do this I wrote my own matrix type that dervies Read and
Show and convert between my custom type and the matrix type from the matrix
library I'm using.
-}
module Util.MatrixParser where

import Data.Matrix
import Control.Exception
import qualified Data.ByteString as BS
import qualified Data.ByteString.Char8 as BSC

import Robotics.ArDrone.NavDataParser

-- | Custom matrix type deriving Read and Show to make parsing and writing easy.
data Matrix3x3 = Matrix3x3 { col1 :: Vector
                           , col2 :: Vector
                           , col3 :: Vector
                           } deriving (Show, Read)

-- | Writes a given matrix to a file at the given location
writeMatrixToFile :: String -> Matrix Float -> IO ()
writeMatrixToFile path matrix = do
  let m3x3 = fromMatrix matrix
  writeFile path $ show m3x3

-- | Reads a matrix from a given file. If the file does not exist the matrix
-- that will be returned will be the 3x3 identity matrix scaled by -1/9.81 and a
-- warning will be printed.
readMatrixFromFile :: String -> IO (Matrix Float)
readMatrixFromFile path = do
  bytes <- (try :: IO BS.ByteString ->IO (Either IOException BS.ByteString)) $ BS.readFile path
  case bytes of
    Left e -> do
      putStrLn "Warning: You didn't create a calibration file with the calibration tool."
      return $ scaleMatrix (-1/(9.81 :: Float)) $ identity 3
    Right b -> do
      let string = BSC.unpack b
      return $ toMatrix $ read string

-- | Conversion function from a matrix to my custom matrix type.
fromMatrix :: Matrix Float -> Matrix3x3
fromMatrix m = Matrix3x3 (Vector (getElem 1 1 m) (getElem 2 1 m) (getElem 3 1 m)) (Vector (getElem 1 2 m) (getElem 2 2 m) (getElem 3 2 m)) (Vector (getElem 1 3 m) (getElem 2 3 m) (getElem 3 3 m))

-- | Conversion from my custom matrix to the matrix library type.
toMatrix :: Matrix3x3 -> Matrix Float
toMatrix m = fromLists $ toRowVectorLists m

-- | Helper function that converts my matrix type to a list of row vectors. 
toRowVectorLists :: Matrix3x3 -> [[Float]]
toRowVectorLists m = [[x $ col1 m, x $ col2 m, x $ col3 m],[y $ col1 m, y $ col2 m, y $ col3 m],[z $ col1 m, z $ col2 m, z $ col3 m]]
