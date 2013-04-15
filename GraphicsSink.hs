module GraphicsSink where

import Graphics.Rendering.OpenGL hiding (Proxy, normalize)
import Graphics.UI.GLUT hiding (Proxy, normalize)
import qualified Graphics.UI.GLUT as GL
import Control.Monad.IO.Class
import Control.Concurrent
import Data.IORef
import Control.Monad
import Pipes
import Linear
import Filter

display :: IORef (Quaternion Double) -> DisplayCallback
display ref = do
    clear [ColorBuffer, DepthBuffer]
    
    quat  <- readIORef ref
    let rot = q2m $ quat
    
    preservingMatrix $ do
        matrixMode $= Modelview 0
        m <- newMatrix RowMajor (map (fromRational . toRational) $ toMatrix rot) :: IO (GLmatrix GLdouble)
        multMatrix m
        renderObject Solid (Teapot 1)
    
    swapBuffers
    
animate :: IdleCallback
animate = do
    threadDelay 10000
    postRedisplay Nothing

lightDiffuse :: Color4 GLfloat
lightDiffuse = Color4 1.0 0.0 0.0 0.0

lightPosition :: Vertex4 GLfloat
lightPosition = Vertex4 1.0 1.0 1.0 0.0

initfn :: IO ()
initfn = let light0 = Light 0
         in do
               matrixMode $= Modelview 0
               preservingMatrix $ do
                   diffuse light0 $= lightDiffuse
                   position light0 $= lightPosition
                   light light0 $= Enabled
                   lighting $= Enabled
               GL.normalize $= Enabled

               depthFunc $= Just Lequal

               matrixMode $= Projection
               perspective 40.0 1.0 1.0 10.0
               matrixMode $= Modelview 0
               lookAt (Vertex3 0.0 0.0 5.0) (Vertex3 0.0 0.0 0.0) (Vector3 0.0 1.0 0.0)

sink :: IORef a -> Consumer a IO r
sink ref = do
    res <- await
    lift $ writeIORef ref res
    sink ref 

initialize :: Quaternion Double -> IO (Consumer (Quaternion Double) IO r)
initialize init = do
    getArgsAndInitialize
    initialDisplayMode $= [DoubleBuffered, RGBMode, WithDepthBuffer]
    createWindow "Red 3D cube"

    ch <- newIORef init 

    displayCallback $= display ch 
    idleCallback $= Just animate 
    initfn

    forkIO mainLoop

    return $ sink ch

