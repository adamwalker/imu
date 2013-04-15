module GaussNewton where

import Debug.TraceUtils

import Numeric.LinearAlgebra hiding (conjugate)
import Linear hiding (trace)
import Control.Lens
import Pipes
import Control.Monad.IO.Class

import ReadSensors

iterat :: Int -> (a -> a) -> a -> a
iterat 0 _    init = init
iterat n func init = iterat (n-1) func (func init)

--We are solving to find the orientation quaternion that takes the measured
--values of gravity and mag to their true values in earth coordinates
--
--We are minimizing the difference between the measurements rotated by the
--orientation quaternion to earth frame and the earth values of gravity and
--mag
solve :: V3 Double -> V3 Double -> V3 Double -> V3 Double -> Matrix Double -> Matrix Double
solve earthMag earthGrav body_grav' body_mag' st = iterat 20 iteration st --(fromLists [[1], [0], [0], [0]])
    where
    iteration :: Matrix Double -> Matrix Double
    iteration guess = fromLists [[a], [b], [c], [d]] `sub` (inv (trans jacob <> jacob) <> (trans jacob <> resid))
        where
        body_grav = normalize body_grav'
        body_mag  = normalize body_mag'
        quat@(Quaternion a (V3 b c d)) = normalize $ Quaternion a' (V3 b' c' d')
            where
            [[a'], [b'], [c'], [d']]  = toLists guess
        resid = fromLists $ map (:[]) $ [resid_grav ^. _x, resid_grav ^. _y, resid_grav ^. _z, resid_mag ^. _x, resid_mag ^. _y, resid_mag ^. _z]
            where
            resid_grav = normalize earthGrav - rotate quat body_grav
            resid_mag  = normalize earthMag  - rotate quat body_mag
        --The jacobian is a 6x4 matrix
        jacob = (-1) * fromBlocks [[jacob_grav], [jacob_mag]]
            where
            jacob_grav = fromBlocks $ [[ja <> v2m body_grav, jb <> v2m body_grav, jc <> v2m body_grav, jd <> v2m body_grav]]
            jacob_mag  = fromBlocks $ [[ja <> v2m body_mag,  jb <> v2m body_mag,  jc <> v2m body_mag,  jd <> v2m body_mag]]
            ja = 2 * fromLists [[ a, -d, c], [d,  a, -b], [-c, b, a]]
            jb = 2 * fromLists [[ b,  c, d], [c, -b, -a], [ d, a, -b]]
            jc = 2 * fromLists [[-c,  b, a], [b,  c,  d], [-a, d, -c]]
            jd = 2 * fromLists [[-d, -a, b], [a, -d,  c], [ b, c,  d]]
        v2m (V3 a b c) = fromLists [[a], [b], [c]]

m2q m = Quaternion a (V3 b c d) 
    where
    [[a], [b], [c], [d]]  = toLists m

doGN :: (Monad m, MonadIO m) => V3 Double -> V3 Double -> Pipe (Double, State) (Double, V3 Double, Quaternion Double) m ()
doGN earthMag earthGrav = func (fromLists [[1], [0], [0], [0]])
    where
    func st = do
        (dt, State accel gyro mag) <- await
        liftIO $ print accel
        liftIO $ print mag
        let res = solve earthMag earthGrav accel mag st
        liftIO $ print res
        yield $ (dt, gyro, normalize $ m2q res)
        func res

