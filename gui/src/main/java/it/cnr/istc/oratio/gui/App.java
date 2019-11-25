package it.cnr.istc.oratio.gui;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Properties;

import javax.swing.UIManager;
import javax.swing.UIManager.LookAndFeelInfo;

import it.cnr.istc.oratio.Context;
import it.cnr.istc.oratio.gui.timelines.PropositionalAgentVisualizer;
import it.cnr.istc.oratio.gui.timelines.PropositionalStateVisualizer;
import it.cnr.istc.oratio.gui.timelines.ReusableResourceVisualizer;
import it.cnr.istc.oratio.gui.timelines.StateVariableVisualizer;

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
            e.printStackTrace();
        }

        if (args.length > 0) {
            StateJInternalFrame state_frame = new StateJInternalFrame(Context.getContext().getCore());
            Context.getContext().addStateListener(state_frame);
            TimelinesJInternalFrame timelines_frame = new TimelinesJInternalFrame(Context.getContext().getCore());
            timelines_frame.addVisualizers(new StateVariableVisualizer(), new ReusableResourceVisualizer(),
                    new PropositionalStateVisualizer(), new PropositionalAgentVisualizer());
            Context.getContext().addStateListener(timelines_frame);

            MainJFrame frame = new MainJFrame(state_frame, timelines_frame);
            frame.setVisible(true);
            try {
                Collection<File> dom_files = new ArrayList<>(args.length - 1);
                for (int i = 0; i < args.length - 1; i++)
                    dom_files.add(new File(args[i]));
                Context.getContext().getCore().read(dom_files.toArray(new File[args.length - 1]));
                Context.getContext().readState(new File(args[args.length - 1]));
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            StateJInternalFrame state_frame = new StateJInternalFrame(Context.getContext().getCore());
            Context.getContext().addStateListener(state_frame);
            TimelinesJInternalFrame timelines_frame = new TimelinesJInternalFrame(Context.getContext().getCore());
            timelines_frame.addVisualizers(new StateVariableVisualizer(), new ReusableResourceVisualizer(),
                    new PropositionalStateVisualizer(), new PropositionalAgentVisualizer());
            Context.getContext().addStateListener(timelines_frame);
            GraphJInternalFrame graph_frame = new GraphJInternalFrame(Context.getContext().getCore());
            Context.getContext().addGraphListener(graph_frame);

            MainJFrame frame = new MainJFrame(state_frame, timelines_frame, graph_frame);
            frame.setVisible(true);

            Properties properties = new Properties();
            try {
                properties.load(App.class.getResourceAsStream("/config.properties"));
            } catch (IOException e) {
                e.printStackTrace();
            }
            Context.getContext().startServer(Integer.parseInt(properties.getProperty("server.port", "1100")));
        }
    }
}
