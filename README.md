ser2sock - Serial to Socket Redirector
======================================

The ser2sock utility allows sharing of a serial device over a TCP/IP
network.  It also supports encryption and authentication via OpenSSL.


Installation
============

1. ./configure
2. make
3. sudo cp ser2sock /usr/local/bin/
4. sudo mkdir -p /etc/ser2sock
5. sudo cp ser2sock.conf.example /etc/ser2sock/ser2sock.conf
6. vi /etc/ser2sock/ser2sock.conf
7. sudo cp init/ser2sock /etc/init.d/
8. sudo update-rc.d ser2sock defaults
9. sudo /etc/init.d/ser2sock start

Usage
=====

```
Usage: ser2sock -p <socket listen port> -s <serial port dev>

  -h, -help                 display this help and exit
  -p port                   socket port to listen on
  -s <serial device>        serial device; ex /dev/ttyUSB0
options
  -i IP                     bind to a specific ip address; default is ALL
  -b baudrate               set baud rate; defaults to 9600
  -d                        daemonize
  -t                        send terminal init string
  -g                        debug level 0-3
  -c                        keep incoming connections when a serial device is disconnected
  -w milliseconds           delay between attempts to open a serial device (5000)
  -e                        use SSL to encrypt the connection
```

Authentication via SSL
======================

In addition to supporting cleartext TCP/IP connections, ser2sock also supports
SSL authentication and encryption as of v1.4.0.

There are three pieces you need to make this work.

* CA certificate - Certificate Authority that is used to authorize clients
  against.
* Server-side certificate - The certificate used by ser2sock, signed by the CA.
* Client-side certificate - The certificate used by clients of ser2sock, also
  signed by the CA.

Generating the Certificates
-------------------------

Create the CA certificate and key:
````openssl req -out ca.pem -new -x509````

Generate the Server key:
```
openssl genrsa -out server.key 2048
```

Generate a signing request for the Server:
```
openssl req -key server.key -new -out server.req
```

Sign the server's request:
```
echo "00" > ca.srl
openssl x509 -req -in server.req -CA ca.pem -CAkey privkey.pem -CAserial ca.srl -out server.pem
```

Generate the Client key:
```
openssl genrsa -out client.key 2048
```

Generate a signing request for the Client:
```
openssl req -key client.key -new -out client.req
```

Sign the client's request:
```
openssl x509 -req -in client.req -CA ca.pem -CAkey privkey.pem -CAserial ca.srl -out client.pem
```

Now all you have to do is enable encryption in the configuration, update your
certificate paths, and restart the service.

Testing the SSL Certificates
---------------------------
After the certificates have been generated and ser2sock restarted you can
verify that the certificates work with OpenSSL:
```
openssl s_client -connect 127.0.0.1:10000 -cert client.pem -key client.key -CAfile ca.pem
```
