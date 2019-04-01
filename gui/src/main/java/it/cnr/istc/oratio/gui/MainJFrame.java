package it.cnr.istc.oratio.gui;

import java.awt.Dimension;

import javax.swing.JFrame;

/**
 * MainJFrame
 */
public class MainJFrame extends JFrame {

    public MainJFrame() {
        super("oRatio");
        setPreferredSize(new Dimension(800, 600));
        setLocationRelativeTo(null);
    }
}