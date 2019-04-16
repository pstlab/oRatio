package it.cnr.istc.oratio.gui;

import javax.swing.DefaultListModel;
import javax.swing.JInternalFrame;
import javax.swing.JList;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Core;

/**
 * LogJInternalFrame
 */
public class LogJInternalFrame extends JInternalFrame implements StateListener {

    private final Core core;
    private final DefaultListModel<String> logs_model = new DefaultListModel<>();
    private final JList<String> logs_list = new JList<>(logs_model);

    public LogJInternalFrame(final Core core) {
        super("Log");
        this.core = core;

        add(new JScrollPane(logs_list));

        setIconifiable(true);
        setMaximizable(true);
        setResizable(true);
        pack();
    }

    @Override
    public void log(String log) {
        SwingUtilities.invokeLater(() -> logs_model.addElement(log));
    }

    @Override
    public void stateChanged(Core core) {
    }
}