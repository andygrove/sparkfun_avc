begin

  filename = ARGV[0]

  puts "<?xml version='1.0' encoding='UTF-8'?>"
  puts "<kml xmlns='http://www.opengis.net/kml/2.2'>"
  puts "  <name>kml_name</name>"
  puts "  <description><![CDATA[]]></description>"
  puts "  <Folder>"
  puts "    <name>folder_name</name>"

  coord = nil
  n = 0
  bearing = nil
  target_bearing = nil
  started = false
  route_info = false
  waypoint_no = 0
  target_waypoint = 0


  File.open(filename, "r").each_line do |line|
    data = line.chomp().split(",")
    #puts data[0]

    if route_info
      if data[0] == 'ROUTE_END'
        route_info = false
      else
        waypoint_no = waypoint_no + 1

        puts "    <Placemark>"
  			puts "      <name>Waypoint #{waypoint_no.to_s}</name>"
        puts "      <description><![CDATA[Compass: #{bearing}. Target bearing: #{target_bearing}]]></description>"
  			puts "      <styleUrl>#waypoint</styleUrl>"
  			puts "      <ExtendedData>"
  			puts "      </ExtendedData>"
  			puts "      <Point>"
  			puts "        <coordinates>#{data[1]},#{data[0]},0.0</coordinates>"
  			puts "      </Point>"
  			puts "    </Placemark>"

      end

    elsif data[0] == 'ROUTE_BEGIN'
      route_info = true

    elsif data[0] == 'START_BUTTON_ACTIVATED'
      started = true

    elsif data[0] == 'WAYPOINT'
      target_waypoint = data[1]

    elsif data[0] == 'BEARING'
      bearing = data[1]
      target_bearing = data[2]

    elsif data[0] == 'GPS'

      if started
        coord = data[2] + "," + data[1]

        n = n + 1

        puts "    <Placemark>"
  			puts "      <name>Point #{n.to_s}</name>"
        puts "      <description><![CDATA[Target waypoint: #{target_waypoint}. Compass: #{bearing}. Target bearing: #{target_bearing}]]></description>"
  			puts "      <styleUrl>#gps</styleUrl>"
  			puts "      <ExtendedData>"
  			puts "      </ExtendedData>"
  			puts "      <Point>"
  			puts "        <coordinates>#{data[2]},#{data[1]},0.0</coordinates>"
  			puts "      </Point>"
  			puts "    </Placemark>"

      end

    end
  end

  puts "  </Folder>"

  puts "<Style id='waypoint'>"
  puts "<IconStyle>"
  puts "<color>ffE8D793</color>"
  puts "<scale>1.1</scale>"
  puts "<Icon>"
  puts "<href>http://www.gstatic.com/mapspro/images/stock/503-wht-blank_maps.png</href>"
  puts "</Icon>"
  puts "<hotSpot x='16' y='31' xunits='pixels' yunits='insetPixels'>"
  puts "</hotSpot>"
  puts "</IconStyle>"
  puts "<LabelStyle>"
  puts "<scale>0.0</scale>"
  puts "</LabelStyle>"
  puts "</Style>"

  puts "<Style id='gps'>"
  puts "<IconStyle>"
  puts "<color>ffDB4436</color>"
  puts "<scale>1.1</scale>"
  puts "<Icon>"
  puts "<href>http://www.gstatic.com/mapspro/images/stock/503-wht-blank_maps.png</href>"
  puts "</Icon>"
  puts "<hotSpot x='16' y='31' xunits='pixels' yunits='insetPixels'>"
  puts "</hotSpot>"
  puts "</IconStyle>"
  puts "<LabelStyle>"
  puts "<scale>0.0</scale>"
  puts "</LabelStyle>"
  puts "</Style>"

  puts "</kml>"

end
