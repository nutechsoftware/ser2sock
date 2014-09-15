ser2sock - Serial to Socket Redirector
======================================

The ser2sock utility allows sharing of a serial device over a TCP/IP
network.  It also supports encryption and authentication via OpenSSL.

05/04/14 Version 1.4.5 adds better support for running more than one copy at a time
on Linux system. init.d script rewritten to provide easy configuration for
multiple instantiations and modifications to ser2sock.c to correct PID file
creation/deletion behavior consistent with running more than one instantiation.

Installation
============
NOTE: The OpenSSL dev package is needed in order to compile with SSL support.

1. ./configure
2. make
3. sudo cp ser2sock /usr/local/bin/
4. sudo cp -R etc/ser2sock /etc/
5. Make changes to /etc/ser2sock/ser2sock.conf as needed.
6. sudo cp init/ser2sock /etc/init.d/
7. sudo update-rc.d ser2sock defaults
8. sudo /etc/init.d/ser2sock start
9. To run more than one instantiation follow instructions in the init/ser2sock script

Installation (Debian/Ubuntu)
============================

1. sudo apt-get install build-essential autotools-dev devscripts
2. debuild -i -us -uc -b
3. sudo dpkg -i ../ser2sock*.deb
4. sudo update-rc.d ser2sock defaults
5. sudo /etc/init.d/ser2sock start
6. To run more than one instantiation follow instructions in the /etc/init.d/ser2sock script

Installation (Mac OS X)
=======================

1. ./configure
2. make
3. mkdir -p /usr/local/bin
4. cp ser2sock /usr/local/bin
5. sudo cp -R etc/ser2sock /etc/
6. Change the 'device' setting in /etc/ser2sock/ser2sock.conf.
7. cp init/ser2sock.plist /System/Library/LaunchAgents/
8. cd ~
9. launchctl load /System/Library/LaunchAgents/ser2sock.plist
10. launchctl start ser2sock

Usage
=====

```
Usage: ./ser2sock -p <socket listen port> -s <serial port dev>

  -h, -help                 display this help and exit
  -f <config pathname>      override config file pathname
  -p port                   socket port to listen on
  -s <serial device>        serial device; ex /dev/ttyUSB0
options
  -i IP                     bind to a specific ip address; default is ALL
  -b baudrate               set baud rate; defaults to 9600
  -d                        daemonize
  -0                        raw device mode - no info messages
  -t                        send terminal init string
  -P <PID file pathname>    overide default PID file pathname. (for use by init.d script)
  -g                        debug level 0-3
  -c                        keep incoming connections when a serial device is disconnected
  -w milliseconds           delay between attempts to open a serial device (5000)
  -e                        use SSL to encrypt the connection
```

Using with more than one serial port (multiple daemon)
======================================================

1. Follow basic installation instructions above.
2. Make an additional copy of the /etc/init.d/ser2sock script for each serial
   port you wish to use but use a unique filename such as /etc/init.d/ser2sock.1 ,
   ser2sock.2 , etc.
3. Edit each file and change the "Provides:" line to use the same name that you
   gave the file: ser2sock.1 (for example)
4. Create a new configuration file in /etc/ser2sock using the same name that you
   gave the init.d script for the corresponding port: ser2sock.1.conf (example)
5. Edit each of those files to reflect the serial device you are using and the
   network port it can be accessed through. Make sure it's a different port from
   the other instantiations and unused by anything else.
6. For each new port do: sudo update-rc.d <script filename> defaults
7. For each new port do: sudo /ect/init.d/<script filename> start


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
---------------------------

Create the CA certificate and key:
```openssl req -out ca.pem -new -x509```

Generate the Server key:
```openssl genrsa -out server.key 2048```

Generate a signing request for the Server:
```openssl req -key server.key -new -out server.req```

Sign the server's request:
```
echo "00" > ca.srl
openssl x509 -req -in server.req -CA ca.pem -CAkey privkey.pem -CAserial ca.srl -out server.pem
```

Generate the Client key:
```openssl genrsa -out client.key 2048```

Generate a signing request for the Client:
```openssl req -key client.key -new -out client.req```

Sign the client's request:
```openssl x509 -req -in client.req -CA ca.pem -CAkey privkey.pem -CAserial ca.srl -out client.pem```

Now all you have to do is enable encryption in the configuration, update your
certificate paths, and restart the service.

Testing the SSL Certificates
----------------------------
After the certificates have been generated and ser2sock restarted you can
verify that the certificates work with OpenSSL:
```openssl s_client -connect 127.0.0.1:10000 -cert client.pem -key client.key -CAfile ca.pem```

Certificate Revocation
----------------------
Certificates may be revoked using OpenSSL's CA facility.

Revoke the certificate:
```openssl ca -config ca.conf -revoke certs/badclient.pem -keyfile certs/privkey.pem -cert certs/ca.pem```

Regenerate the CRL:
```openssl ca -config ca.conf -gencrl -keyfile certs/privkey.pem -cert certs/ca.pem -out ser2sock.crl```

Restart ser2sock:
```killall -HUP ser2sock```
