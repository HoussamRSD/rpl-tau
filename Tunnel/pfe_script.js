/*
 * Improved Contiki test script (JavaScript).
 * A Contiki test script acts on mote output, such as via printf()'s.
 * This script only logs the lines strictly required by the analyze.py script 
 * to save disk space and improve the simulation speed.
 */

/* 60 minutes timeout. Stops the simulation gracefully with testOK() */
TIMEOUT(3600000, log.testOK());

while (true) {
  if (msg) {
    /* Filter to only keep messages needed for evaluation metrics */
    if (msg.indexOf("Client sending") != -1 ||
        msg.indexOf("Server received") != -1 ||
        msg.indexOf("Sending a multicast-DIO") != -1 ||
        msg.indexOf("Sending unicast-DIO") != -1 ||
        msg.indexOf("Sending a DAO") != -1 ||
        msg.indexOf("Sending a No-Path DAO") != -1 ||
        msg.indexOf("Sending a DIS") != -1 ||
        msg.indexOf("#A Parent Switch!") != -1 ||
        msg.indexOf("Energest:") != -1) {
          
        log.log(time + ":" + id + ":" + msg + "\n");
    }
  }
  YIELD();
}
