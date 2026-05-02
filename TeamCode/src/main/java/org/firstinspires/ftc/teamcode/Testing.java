package org.firstinspires.ftc.teamcode;

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.SocketException;
import java.io.IOException;

public class Testing {
    public static void main(String[] args) {
        try {
            DatagramSocket socket = new DatagramSocket(5555);

            byte[] buffer = new byte[1024];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            socket.receive(packet);

            String data = new String(packet.getData(), 0, packet.getLength());
            System.out.println(data);

            socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}