filename = ARGV[0]

coord = nil
n = 1

File.open(filename, "r").each_line do |line|
  data = line.split(",")
  if data[1] == 'GPS'
    coord = data[2] + "," + data[3]
  elsif data[1] == 'EVENT' 
    unless coord.nil?
      puts coord.chomp + ",Point" + n.to_s + "_" + data[2]
      n = n + 1
    end
  end
end
