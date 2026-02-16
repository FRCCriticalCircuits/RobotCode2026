package frc.robot.utils;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;

public class AxisConfigLoader {
    private static final HashMap<String, AxisMappingTable> map = new HashMap<>();

    private static AxisMappingTable loadFromFile(String configName) throws IOException {
        String path = "axis_configs/" + "AxisConfig_" + configName + ".json";

        AxisMappingTable tableBuffer = new AxisMappingTable();
        
        JsonFactory factory = new JsonFactory();
        JsonParser parser = factory.createParser(new File(Filesystem.getDeployDirectory(), path));

        double x = 0, y = 0;

        while (parser.nextToken() != null) {
            if (parser.currentToken() == JsonToken.FIELD_NAME) {
                String fieldName = parser.currentName();

                if ("x".equals(fieldName)) {
                    parser.nextToken();
                    x = parser.getDoubleValue();
                }

                if ("y".equals(fieldName)) {
                    parser.nextToken();
                    y = parser.getDoubleValue();

                    // Insert into interpolation table
                    tableBuffer.put(x / 100.0, y / 100.0);
                }
            }
        }

        parser.close();
        map.put(configName, tableBuffer);

        return tableBuffer;
    }
    
    public static AxisMappingTable loadTable(String configName){
        if(map.get(configName) != null) return map.get(configName);

        try {
            return loadFromFile(configName);
        } catch (IOException e) {
            System.err.println("Failed to load axis config: " + configName);
            e.printStackTrace();

            AxisMappingTable template = new AxisMappingTable();
            template.put(0.0, 0.0);
            template.put(0.2, 0.0);
            template.put(1.0, 0.2);

            return template;
        }        
    }
}
