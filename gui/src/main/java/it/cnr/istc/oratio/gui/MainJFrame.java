package it.cnr.istc.oratio.gui;

import java.awt.Dimension;
import java.awt.Frame;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.beans.PropertyVetoException;
import java.util.Arrays;
import java.util.stream.Stream;

import javax.swing.ImageIcon;
import javax.swing.JDesktopPane;
import javax.swing.JFrame;
import javax.swing.JInternalFrame;
import javax.swing.WindowConstants;

/**
 * MainJFrame
 */
public class MainJFrame extends JFrame {

    private final JDesktopPane mainJDesktopPane = new JDesktopPane();

    public MainJFrame(JInternalFrame... frames) {
        super("oRatio");

        for (JInternalFrame frame : frames) {
            frame.setVisible(true);
            mainJDesktopPane.add(frame);
        }

        add(mainJDesktopPane);

        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                // How many frames do we have?
                JInternalFrame[] allframes = Stream.of(mainJDesktopPane.getAllFrames()).filter(frame -> !frame.isIcon())
                        .toArray(JInternalFrame[]::new);
                int count = allframes.length;
                if (count == 0) {
                    return;
                }

                // Determine the necessary grid size
                int sqrt = (int) Math.sqrt(count);
                int rows = sqrt;
                int cols = sqrt;
                if (rows * cols < count) {
                    cols++;
                    if (rows * cols < count) {
                        rows++;
                    }
                }

                // Define some initial values for size & location.
                Dimension size = mainJDesktopPane.getSize();

                int w = size.width / cols;
                int h = size.height / rows;
                int x = 0;
                int y = 0;

                // Iterate over the frames, deiconifying any iconified frames and then
                // relocating & resizing each.
                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols && ((i * cols) + j < count); j++) {
                        JInternalFrame f = allframes[(i * cols) + j];

                        if (!f.isClosed() && f.isIcon()) {
                            try {
                                f.setIcon(false);
                            } catch (PropertyVetoException ignored) {
                            }
                        }

                        mainJDesktopPane.getDesktopManager().resizeFrame(f, x, y, w, h);
                        x += w;
                    }
                    y += h; // start the next row
                    x = 0;
                }
            }
        });

        setExtendedState(Frame.MAXIMIZED_BOTH);
        setIconImages(Arrays.asList(new ImageIcon(getClass().getResource("/oRatio16.png")).getImage(),
                new ImageIcon(getClass().getResource("/oRatio32.png")).getImage()));
        setPreferredSize(new Dimension(800, 600));
        pack();
        setLocationRelativeTo(null);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
    }
}