package it.cnr.istc.oratio.gui;

import java.io.IOException;
import java.util.Properties;

import javax.swing.UIManager;
import javax.swing.UIManager.LookAndFeelInfo;

import it.cnr.istc.oratio.Context;

/**
 * Hello world!
 *
 */
public class App {

    public static void main(String[] args) {
        try {
            for (LookAndFeelInfo info : UIManager.getInstalledLookAndFeels())
                if ("Nimbus".equals(info.getName())) {
                    UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
        } catch (Exception e) {
            // If Nimbus is not available, you can set the GUI to another look and feel.
        }

        Properties properties = new Properties();
        try {
            properties.load(App.class.getResourceAsStream("/config.properties"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        MainJFrame frame = new MainJFrame();
        frame.setVisible(true);

        Context.getContext().startServer(Integer.parseInt(properties.getProperty("server.port", "1100")));
    }
}
