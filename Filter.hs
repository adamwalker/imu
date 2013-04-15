module Filter where

import Control.Monad.IO.Class

import Debug.Trace
import Debug.TraceUtils

import Linear hiding (trace)
import Numeric.LinearAlgebra hiding (conjugate)
import Numeric.LinearAlgebra.Util hiding (norm, cross)
import Pipes

import ReadSensors (State(..))

--variances
var_pos_init    = 0
var_upd_ang_vel = 1000
var_upd_orient  = 0.1 --0.02
var_meas_orient = 5
var_meas_angvel = 0.5

state_orient_init = [1, 0, 0, 0]
state_angvel_init = [0, 0, 0]

{-
    The quaternion state takes body measurements to earth measurements

    The angular velocity is measured in the body frame

    q' ~= q + dt * dq/dt
    
    when the angular velocity is measured in body coordinates the derivative is:
    dq/dt = 1/2 * q * w
    q' ~= q + dt * 1/2 * q * w
    
    w' = w + 1/2 * dt * (w0 - xp - yq - zr)
    x' = x + 1/2 * dt * (wp + x0 + yr - zq)
    y' = y + 1/2 * dt * (wq - xr + y0 + zp)
    z' = z + 1/2 * dt * (wr + xq - yp + z0)

-}

{-
    q * w
    (w, x, y, z) * (0, p, q, r) 

    w0 - xp - yq - zr
    wp + x0 + yr - zq
    wq - xr + y0 + zp
    wr + xq - yp + z0
-}

{-
    w * q
    (0, p, q, r) * (w, x, y, z) 

    0w - px - qy - rz
    0x + pw + qz - ry
    0y - pz + qw + rx
    0z + py - qx + rw
-}

{-
updateOrient orient angvel dt = orient ^+^ (derivative ^* dt)
    where
    derivative = 0.5 * orient * angvel
-}

updateOrient orient angvel dt = theQ * orient
    where
    Quaternion _ eAngVel = orient * angvel * conjugate orient
    theta = (dt * norm eAngVel) / 2
    nav   = eAngVel ^/ norm eAngVel
    theQ  = if norm eAngVel == 0 then Quaternion 1 (V3 0 0 0) else Quaternion (cos theta) (sin theta *^ nav)

computeJacobian dt state = fromBlocks [[f_orient_orient, f_orient_angvel], [f_angvel_orient, f_angvel_angvel]]
    where
    [w, x, y, z, p, q, r] = toList state
    --[oo, oa]
    --[0,  aa]
    f_orient_orient = ident 4 + (0.5 * dt) `scale` fromLists [
            [0, -p, -q, -r], --dw/dw, dw/dx, dw/dy, dw/dz
            [p,  0,  r, -q], --dx/dw, dx/dx, dx/dy, dx/dz
            [q, -r,  0,  p], --dy/dw, dy/dx, dy/dy, dy/dz
            [r,  q, -p,  0]  --dz/dw, dz/dx, dz/dy, dz/dz
        ]
    f_orient_angvel = (0.5 * dt) `scale` fromLists [
            [-x, -y, -z],   --dw/dp, dw/dq, dw/dr
            [ w, -z,  y],   --dx/dp, dx/dq, dx/dr
            [ z,  w, -x],   --dy/dp, dy/dq, dy/dr
            [-y,  x,  w]    --dz/dp, dz/dq, dz/dr
        ]
    f_angvel_orient = zeros 3 4
    f_angvel_angvel = ident 3

h = ident 7

q = diagl $ replicate 3 var_upd_ang_vel
r = diagl $ replicate 4 var_meas_orient ++ replicate 3 var_meas_angvel
q' = diagl $ replicate 4 0 ++ replicate 3 var_upd_ang_vel

computeL state dt = fromBlocks [[q_orient_accel], [q_angvel_accel]]
    where
    [w, x, y, z, p, q, r] = toList state
    q_orient_accel = (0.25 * dt * dt) `scale` fromLists [
            [-x, -y, -z],   --dw/dp, dw/dq, dw/dr
            [ w, -z,  y],   --dx/dp, dx/dq, dx/dr
            [ z,  w, -x],   --dy/dp, dy/dq, dy/dr
            [-y,  x,  w]    --dz/dp, dz/dq, dz/dr
        ]
    q_angvel_accel = dt `scale` ident 3

updateState state dt = fromList [w', x', y', z', p, q, r]
    where
    [w, x, y, z, p, q, r] = toList state
    Quaternion w' (V3 x' y' z') = normalize $ updateOrient (Quaternion w (V3 x y z)) (Quaternion 0 (V3 p q r)) dt

data KState = KState {
    state :: Vector Double,
    cov   :: Matrix Double
}

initKState = KState (fromList [1, 0, 0, 0, 0, 0, 0]) (diagl $ replicate 4 var_pos_init ++ replicate 3 0)

kalman :: KState -> Vector Double -> Double -> KState 
kalman (KState state cov) meas dt = KState next_state next_cov
    where
    jacob      = computeJacobian dt state 
    l          = computeL state dt
    pred_state = updateState state dt
    pred_cov   = (jacob <> cov <> trans jacob) `add` (l <>  q <> trans l) `add` fromBlocks [[var_upd_orient * dt `scale` ident 4, zeros 4 3], [zeros 3 4, zeros 3 3]]
    meas_resid = meas `sub` (h <> pred_state)
    cov_resid  = (h <> pred_cov <> trans h) `add` r
    k_gain     = trans $ linearSolve (trans cov_resid) (trans (pred_cov <> trans h))
    next_state = pred_state `add` (k_gain `mXv` meas_resid)
    next_cov   = (ident 7 `sub` (k_gain <> h)) <> pred_cov

filt :: (MonadIO m) => Pipe (Double, V3 Double, Quaternion Double) (Quaternion Double) m ()
filt = func initKState
    where 
    func st = do
        --liftIO $ print $ cov st
        (dt, gyro, orient) <- await
        let lastQuart = listToQ $ take 4 $ toList $ state st
        let meas = fromList $ (q2list $ closeQuart lastQuart orient) ++ v3ToList gyro
        let res = kalman st meas dt
        yield $ listToQ $ take 4 $ toList $ state res
        func res

toMatrix :: (Num a) => V3 (V3 a) -> [a]
toMatrix (V3 (V3 a b c) (V3 d e f) (V3 g h i)) = [a, b, c, 0, d, e, f, 0, g, h, i, 0, 0, 0, 0, 1]

q2list :: Quaternion Double -> [Double]
q2list (Quaternion x (V3 y z w)) = [x, y, z, w]

listToQ :: [Double] -> Quaternion Double
listToQ [x, y, z, w] = Quaternion x (V3 y z w)

v3ToList :: V3 Double -> [Double]
v3ToList (V3 x y z) = [x, y, z]

q2m :: Quaternion Double -> V3 (V3 Double)
q2m (Quaternion w (V3 x y z)) = V3 
    (V3 (1 - 2*y*y - 2*z*z) (2*x*y - 2*z*w) (2*x*z + 2*y*w))
    (V3 (2*x*y + 2*z*w) (1 - 2*x*x - 2*z*z) (2*y*z - 2*x*w))
    (V3 (2*x*z - 2*y*w) (2*y*z + 2*x*w) (1 - 2*x*x - 2*y*y))

closeQuart :: Quaternion Double -> Quaternion Double -> Quaternion Double
closeQuart old new = if d1 > d2 then new else (- new)
    where
    d1 = old `Linear.dot` new 
    d2 = old `Linear.dot` (- new)

