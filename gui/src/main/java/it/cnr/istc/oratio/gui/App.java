/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.oratio.gui;

import com.google.gson.Gson;
import java.awt.Dimension;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class App {

    private static final String FLAW_CREATED = "flaw_created ";
    private static final String FLAW_STATE_CHANGED = "flaw_state_changed ";
    private static final String CURRENT_FLAW = "current_flaw ";
    private static final String RESOLVER_CREATED = "resolver_created ";
    private static final String RESOLVER_STATE_CHANGED = "resolver_state_changed ";
    private static final String RESOLVER_COST_CHANGED = "resolver_cost_changed ";
    private static final String CURRENT_RESOLVER = "current_resolver ";
    private static final String CAUSAL_LINK_ADDED = "causal_link_added ";
    private static final String STATE_CHANGED = "state_changed ";

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        Gson gson = new Gson();
        CausalGraph cg = new CausalGraph();
        JFrame frame = new JFrame("oRatio");
        frame.setSize(new Dimension(800, 600));
        frame.setLocationRelativeTo(null);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(cg);
        SwingUtilities.invokeLater(() -> frame.setVisible(true));
        try {
            System.out.println("starting GUI server..");
            ServerSocket serverSocket = new ServerSocket(1100);
            Socket clientSocket = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream(), "UTF-8"));
            String inputLine;
            while ((inputLine = in.readLine()) != null) {
                if (inputLine.startsWith(FLAW_CREATED)) {
                    cg.flaw_created(gson.fromJson(inputLine.substring(FLAW_CREATED.length()), CausalGraph.FlawCreated.class));
                } else if (inputLine.startsWith(FLAW_STATE_CHANGED)) {
                    cg.flaw_state_changed(gson.fromJson(inputLine.substring(FLAW_STATE_CHANGED.length()), CausalGraph.FlawStateChanged.class));
                } else if (inputLine.startsWith(CURRENT_FLAW)) {
                    cg.current_flaw(gson.fromJson(inputLine.substring(CURRENT_FLAW.length()), CausalGraph.CurrentFlaw.class));
                } else if (inputLine.startsWith(RESOLVER_CREATED)) {
                    cg.resolver_created(gson.fromJson(inputLine.substring(RESOLVER_CREATED.length()), CausalGraph.ResolverCreated.class));
                } else if (inputLine.startsWith(RESOLVER_STATE_CHANGED)) {
                    cg.resolver_state_changed(gson.fromJson(inputLine.substring(RESOLVER_STATE_CHANGED.length()), CausalGraph.ResolverStateChanged.class));
                } else if (inputLine.startsWith(RESOLVER_COST_CHANGED)) {
                    cg.resolver_cost_changed(gson.fromJson(inputLine.substring(RESOLVER_COST_CHANGED.length()), CausalGraph.ResolverCostChanged.class));
                } else if (inputLine.startsWith(CURRENT_RESOLVER)) {
                    cg.current_resolver(gson.fromJson(inputLine.substring(CURRENT_RESOLVER.length()), CausalGraph.CurrentResolver.class));
                } else if (inputLine.startsWith(CAUSAL_LINK_ADDED)) {
                    cg.causal_link_added(gson.fromJson(inputLine.substring(CAUSAL_LINK_ADDED.length()), CausalGraph.CausalLinkAdded.class));
                } else if (inputLine.startsWith(STATE_CHANGED)) {
                    State state = gson.fromJson(inputLine.substring(CAUSAL_LINK_ADDED.length()), State.class);
                }
            }
            serverSocket.close();
        } catch (IOException ex) {
            Logger.getLogger(App.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
