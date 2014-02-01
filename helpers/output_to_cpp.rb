filename = ARGV[0]
raw_output = File.readlines(filename)
data = raw_output.map { |l| l.split(" ").map(&:to_f) }.transpose
file_partial = filename.split('.')[0]
file_partial = filename[0].upcase + file_partial[1..-1]

puts data.each_with_index.map { |dim, i|
  d = ['X', 'Y', 'Z'][i]
  "double raw#{file_partial}Swipe#{d}[#{data[0].size}] = {#{dim.join(', ')}};"
}
