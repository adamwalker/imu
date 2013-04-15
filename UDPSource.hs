module UDPSource where

import Network.Socket
import Control.Monad 
import Pipes

prodUDP :: Producer String IO ()
prodUDP = do
    s        <- lift $ socket AF_INET Datagram defaultProtocol
    bindAddr <- lift $ inet_addr "0.0.0.0"
    lift $ bindSocket s (SockAddrInet 5555 bindAddr)
    forever $ do
        (msg,len,from) <- lift $ recvFrom s 1000
        yield msg

