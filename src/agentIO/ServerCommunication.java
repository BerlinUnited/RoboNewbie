/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Monika Domanska
 * @version 1.1 Changed by hdb 26.2.2014, 14.5.2014, 16.10.2014
 * 
 * Package agentIO uses code from magmaOffenburg.
 * http://robocup.fh-offenburg.de/
*******************************************************************************/
package agentIO;

import agentIO.perceptors.GameStatePerceptor;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import util.BeamPoses;

/**
 * Connection to the SimSpark server, receiving and sending of messages via TCP.
 * <p>
 * This class encapsulates the network protocol of SimSpark. For users of
 * RoboNewbie it is not necessary, to know anything about that protocol.
 * <p>
 * Just use this class as follows: <br>
 * An agent has always exactly one instance of this class. <br>
 * 1) The constructor establishes a connection to the server at the address
 * 127.0.0.1 (localhost). (If the agent should connect to a server with another
 * IP, change the variable "host" of this class.) <br>
 * 2) Pass the instance of this class to the constructors of PercepTorInput and
 * EffectorOutput. <br>
 * 3) Init the robot with method initRobot(...). <br>
 * 4) Avoid using methods getServerMessage() and sendAgentMessage(...), if you
 * don´t know exactly, what you are doing. Use classes PerceptorInput and
 * EffectorOutput!
 * <p>
 * The typical usage of this class together with classes PerceptorInput and
 * EffectorOutput is shown in Agent_BasicStructure in package examples.
 * <p>
 * <p/>
 * Information for developers of RoboNewbie (students actually learning robotics
 * dont need that):
 * <p/>
 * This class realises the sending and receiving of SimSpark messages.
 * <p/>
 * During every cycle of the main agent loop the actual server message has to be
 * read by calling the method getServerMessage(). This synchronizes the agent
 * with the server, and should be done by an object of class PerceptorInput. So
 * the call stack would be:<br>
 * Agent_-class.sense() <br>
 * -> PerceptorInput.update() <br>
 * -> ServerConnection.getServerMessage()
 * <p/>
 * With sendMessage() an agent message can be sent. Also once during the main
 * agent loop, this should be done by class EffectorOutput. Call stack: <br>
 * Agent_-class.act() <br>
 * -> EffectorOutput.sendAgentMessage(); <br>
 * -> ServerConnection.sendAgentMessage();
 * <p/>
 * The source code is partly copied from the RoboCup-Team magmaOffenburg. <br>
 * http://robocup.fh-offenburg.de/html/downloads.htm , downloaded at 14.1.2012.
 * <br>
 * Path in the source directory: <br>
 * src\magma\agent\connection\impl\ServerConnection.java
 * <p/>
 */
public class ServerCommunication {

    private String host = "127.0.0.1";
    private int port = 3100;
    private DataInputStream in;
    private DataOutputStream out;
    private Socket socket;

    PerceptorInput percIn;

    /**
     * Parameters that specify the robot finally after initialization. Some have
     * been assigned by the Server if id=0 was specified for the initialization
     * of the agent. .
     *
     * In future implementations they may be contained in a special class
     * providing further information about the robot itself, e.g. by the vision
     * perceptor. Up to now, this information is not provided by LocalFieldView.
     * Hinge joint perceptors might also be included in such a new class.
     */
    public String teamSide;
    public String teamName;
    public String finalRobotID;
    public double finalBeamCoordsX;
    public double finalBeamCoordsY;
    public double finalBeamCoordsRot;

    /**
     * Constructor, establishes the TCP-connection to the server.
     */
    public ServerCommunication() {
        try {
            socket = new Socket(host, port);
            socket.setTcpNoDelay(true);

            in = new DataInputStream(socket.getInputStream()); //new DataInputStream(socket.getInputStream());
            out = new DataOutputStream(socket.getOutputStream());

            System.out.println("Connection to: " + host + ":" + port);

        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (ConnectException e) {
            System.out.println(host
                    + ":"
                    + port
                    + " refused the connection. Is rcssserver3d running? Are you using an IPv6-enabled"
                    + " system and the host name translates to an IPv6 address?");
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * This method initializes the robot on the soccer field.
     * <p/>
     * The robot is initialized as a player of a certain team with a certain
     * player number, and it is beamed at its initial position on the field.
     * <br>
     * If the player number is specified as "0", the server assigns a number by
     * the order of initialization. This allows to use the same agent program
     * for different players. Beam positions are then taken from
     * util.BeamPoses.<br>
     * After initialization, this method receives some first messages from the
     * server, because the gyrometer and the accelerometer need the first server
     * cycles to even out. (actually suspended for speeding up -hdb)
     * <p/>
     * Caution: When the server runs in agent sync mode, this method cannot be
     * debugged line by line. It has to be executed as a whole, and after that
     * every breakpoint is possible.
     *
     * @param robotID The ID the robot should have, this is a string containing
     * just a number from 1 to 11, like: "3". In case "0", the numbers are
     * assigned by the server during initialization.
     * @param team The custom team name of the robot. All agents in one team
     * (playing to the same side of the field) must have exactly the same team.
     * The server allows agents of two teams on the field. If an agent tries to
     * initialize its robot with a third team name, this will not work.
     * @param beamCoordsX X-coordinate for the initial position.
     * @param beamCoordsY Y-coordinate for the initial position.
     * @param beamCoordsRot Angle to the direction, which the robot should face
     * at its initial position.
     */
    public void initRobot(String robotID, String team, double beamCoordsX, double beamCoordsY, double beamCoordsRot) {
        sendAgentMessage("(scene rsg/agent/nao/nao.rsg)(syn)");
        getServerMessage();
        
        sendAgentMessage("(init (unum " + robotID + ")(teamname " + team + "))(syn)");
        getServerMessage();
        
        PerceptorInput perc = new PerceptorInput(this);
        /* after initializing, the next ServerMessage (and only this message)
         * contains information about team side (left/right) and player number
         * in the game state: (GS (unum X) (team Y) ... ).
         * Team side is needed for direction of attack.
         * The following call of perc.update() is used for analyzing.
         * Changed by hdb 26.2.2014
         *
         * Heinrich, 09.06.2016
         * In sync mode the simulator is waiting for "(sync)" messages before it 
         * sends a response.
         */
        sendAgentMessage("(syn)");
        perc.update();
        GameStatePerceptor firstGameState = perc.getGameState();

        teamSide = firstGameState.getTeamSide();
        teamName = team;

        if (robotID.equals("0")) {
            finalRobotID = firstGameState.getAgentNumber();
            int i = Integer.parseInt(finalRobotID);

            finalBeamCoordsX = BeamPoses.getX(i);
            finalBeamCoordsY = BeamPoses.getY(i);
            finalBeamCoordsRot = BeamPoses.getRot(i);
        } else {
            finalRobotID = robotID;
            finalBeamCoordsX = beamCoordsX;
            finalBeamCoordsY = beamCoordsY;
            finalBeamCoordsRot = beamCoordsRot;
        }

        /*
         * Beam the robot to the specified pose.
         */
        sendBeamMessage();

        // TODO: why do we need this?
        for (int i = 0; i < 100; i++) {
            getServerMessage();
            sendAgentMessage("(syn)");
        }
    }

    /**
     * Beam the player to a given pose. Used at the beginning and after goals.
     *
     * If player number was assigned by the server, the beam-message uses the
     * poses from class util.BeamPoses. Otherwise the parameters from calling
     * initRobot are used. Changed by hdb 26.2.2014, 16.10.2014
     */
    public void sendBeamMessage() {
        sendAgentMessage("(beam " + finalBeamCoordsX + " " + finalBeamCoordsY + " " + finalBeamCoordsRot + ")(syn)");
    }

    /**
     * Sends an agent message to the server.
     * <p/>
     * This method formats an agent message (String of SimSpark effector
     * messages) according to the network protocol and sends it to the server.
     * <p/>
     * The content of the agent message is not validated.
     *
     * @param msg Agent message with effector commands.
     */
    public void sendAgentMessage(String msg) {

        byte[] body = msg.getBytes();

        //comments by the authors of magma from Offenburg:
        // FIXME: this is to compensate a server bug that clients responding too
        // quickly get problems
        // long runtime = 0;
        // boolean slowedDown = false;
        // long slowDownTime = 0;
        // int minWaitTime = 1000000;
        // do {
        // runtime = System.nanoTime() - startTime;
        // if (runtime < minWaitTime && !slowedDown) {
        // slowDownTime = minWaitTime - runtime;
        // slowedDown = true;
        // }
        // } while (runtime < minWaitTime);
        // if (slowedDown) {
        // logger.log(Level.FINE, "slowedDown sending message by: {0}",
        // slowDownTime);
        // }
        // Header of the message, specifies the length of the message:
        // "The length prefix is a 32 bit unsigned integer in network order, i.e. big 
        // endian notation with the most significant bits transferred first." 
        // (cited from 
        // http://simspark.sourceforge.net/wiki/index.php/Network_Protocol, 14.1.2012)
        // NOTE: DataOutputStream.writeInt allways writes bytes in big endian
        try {
            out.writeInt(body.length);
            out.write(body);
            out.flush();
        } catch (IOException e) {
            throw new ConectionException("Error writing to socket. Has the server been shut down?", e);
        }
    }

    /**
     * Receives a server message and returns it.
     * <p/>
     * This method listens (blocking) for the next SimSpark message from the
     * server, removes the header concerning the SimSpark network protocol and
     * returns the server message (String of perceptor messages). <br>
     * If the server has sent more then one message since the last call of this
     * method, the oldest is returned, that means the messages are provided
     * always in chronological order.
     * <p/>
     * @return The raw server message (String of concatenated perceptor
     * messages).
     */
    public String getServerMessage() {

        try {
            // read the length of the following message  
            int length = in.readInt();
            if (length < 0) {
                throw new ConectionException("Server ist down.");
            }

            // read the message
            byte[] result = new byte[length];
            int total = 0;
            while (total < length) {
                total += in.read(result, total, length - total);
            }

            return new String(result, 0, length, "UTF-8");
        } catch (IOException e) {
            throw new ConectionException("Error when reading from socket. Has the server been shut down?", e);
        }
    }

    /**
     * Exception thrown when the connection was closed by the server.
     * @see getServerMessage()
     * @see sendAgentMessage(String msg)
     */
    public static class ConectionException extends RuntimeException {

        public ConectionException() {
            super();
        }

        public ConectionException(String message) {
            super(message);
        }

        public ConectionException(String message, Throwable cause) {
            super(message, cause);
        }

        public ConectionException(Throwable cause) {
            super(cause);
        }
    }
}
