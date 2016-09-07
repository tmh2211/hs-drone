module Util.MatrixParser where

import Data.Matrix
import Control.Exception
import qualified Data.ByteString as BS
import qualified Data.ByteString.Char8 as BSC

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
  bytes <- (try :: IO BS.ByteString ->IO (Either IOException BS.ByteString)) $ BS.readFile path
  case bytes of
    Left e -> do
      putStrLn "Warning: You didn't create a calibration file with the calibration tool."
      return $ scaleMatrix (-1/(9.81 :: Float)) $ identity 3
    Right b -> do
      let string = BSC.unpack b
      return $ toMatrix $ read string

fromMatrix :: Matrix Float -> Matrix3x3
fromMatrix m = Matrix3x3 (Vector (getElem 1 1 m) (getElem 2 1 m) (getElem 3 1 m)) (Vector (getElem 1 2 m) (getElem 2 2 m) (getElem 3 2 m)) (Vector (getElem 1 3 m) (getElem 2 3 m) (getElem 3 3 m))

toMatrix :: Matrix3x3 -> Matrix Float
toMatrix m = fromLists $ toRowVectorLists m

toRowVectorLists :: Matrix3x3 -> [[Float]]
toRowVectorLists m = [[x $ col1 m, x $ col2 m, x $ col3 m],[y $ col1 m, y $ col2 m, y $ col3 m],[z $ col1 m, z $ col2 m, z $ col3 m]]
