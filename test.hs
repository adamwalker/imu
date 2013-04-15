import Pipes as P
import Pipes.Prelude as P

import ReadSensors hiding (scale, normalize)
import qualified ReadSensors
import Filter
import GraphicsSink
--import Control
import Linear
import UDPSource
import GaussNewton

skipFilter :: Monad m => Pipe (Double, V3 Double, Quaternion Double) (Quaternion Double) m ()
skipFilter = do
    (dt, mag, orient) <- await
    yield (orient)
    skipFilter

sensors :: Pipe Result Void IO ()
sensors = do
    mag  <- calibrate Mag   20
    acc  <- calibrate Accel 20
    sink <- liftIO $ initialize (Quaternion 1 (V3 0 0 0))
    procMsg >-> doGN mag acc >-> filt >-> sink

main :: IO ()
main = do
    runEffect $ prodUDP >-> P.map process >-> sensors

