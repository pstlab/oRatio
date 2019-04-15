package it.cnr.istc.oratio.gui;

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

        MainJFrame frame = new MainJFrame();
        frame.setVisible(true);

        Context.getContext().startServer();
    }
}
