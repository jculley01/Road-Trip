/* Quest 4: Road Trip
   BU ENG EC 444 Spring 2023
   Team 2: John Culley, Ananth Sanjay, Jeffrey Zhang, Kai Imery
   04/04/2023 
*/
var fs = require('fs'); //module to handle the file system
var express = require('express'); //webserver module
var bodyParser = require('body-parser'); //parse JSON encoded strings
var app = express();
var debug = require('debug')('myserver'); //debug module

/* Constants */
const HIGH = "1", LOW = "0", IP_ADDR = '0.0.0.0', PORT_ADDR = 8082;
const UDP = require('dgram')

const client = UDP.createSocket('udp4')

const port = 3333

const hostname = '192.168.1.29'

//Using Express to create a server
app.use(express.static(__dirname)); //add the directory where HTML and CSS files are
var server = app.listen(PORT_ADDR, IP_ADDR, function () {//listen at the port and address defined
    var host = server.address().address;
    var port = server.address().port;
    var family = server.address().family;
    debug('Express server listening at http://%s:%s %s', host, port, family);
});

app.use(bodyParser.urlencoded({ //to support URL-encoded bodies, must come before routing
    extended: true
}));

app.route('/gpio')
    .post(function (req, res) { //handles incoming POST requests
        var serverResponse = { status: '' };
        var btn = req.body.id, val = req.body.val; // get the button id and value
        if (val == 'on') { //if button is clicked, turn on the leds
            sendMess(HIGH);
        }
        else { //if button is unclicked, turn off the leds
            sendMess(LOW);
        }
    });

function sendMess(message) {
    /*---------- export pin if not exported and configure the pin direction -----------*/
    client.on('message', (message, info) => {
        // get the information about server address, port, and size of packet received.

        console.log('Address: ', info.address, 'Port: ', info.port, 'Size: ', info.size)

        //read message from server

        console.log('Message from server', message.toString())
    })

    const packet = Buffer.from(message)

    client.send(packet, port, hostname, (err) => {
        if (err) {
            console.error('Failed to send packet !!')
        } else {
            console.log('Packet send !!')
        }
    })
}