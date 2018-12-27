/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package it.cnr.istc.oratio.gui.riddle;

import java.io.File;
import org.junit.jupiter.api.Test;

/**
 *
 * @author ricde
 */
public class CoreTest {

    @Test
    public void testCore0() throws Exception {
        Core instance = new Core();
        instance.read(new File[] { new File("../examples/test_rr_0.rddl") });
    }
}
