function tests = dijkstra_test
tests = functiontests(localfunctions);
end

function assertPathValid(testcase, map, path, start, stop, xlim, ylim, zlim)
valid = xlim(1) <= path(:, 1) & path(:, 1) <= xlim(2);
valid = ylim(1) <= path(:, 2) & path(:, 2) <= ylim(2) & valid;
valid = zlim(1) <= path(:, 3) & path(:, 3) <= zlim(2) & valid;
verifyTrue(testcase, all(valid));

c = collide(map, path);
verifyFalse(testcase, any(c));

verifyEqual(testcase, path(1, :), start);
verifyEqual(testcase, path(end, :), stop);
end
%{d
function testMap1(testcase)
map = load_map('../sample_maps/map0.txt', 0.1, 2.0, 0.3);
%start = [10 20 6];
%start = [0 20 6];
%start = [0 -5 0];
start = [0.0  -4.9 0.2];
%stop = map.boundary(4:6);
stop  = [8.0  18.0 3.0];
%stop = [10 -5 0];
[path, nodes_expanded] = dijkstrav4(map, start, stop, true);
nodes_expanded
size(path)
%plot_path(map,path);
assertPathValid(testcase, map, path, start, stop, [0 10], [-5 20], [0 6]);
end
%}
%{
function testEmptyMap(testcase)
map = load_map('../sample_maps/emptyMap.txt', 0.25, 10.0, 4.0)
%start = [1.81 2.40, 5.5];
start = [10 0 6];
%stop = map.boundary(4:6);
stop = [10 10 0];
%stop = [0 10 0];
%stop = [9.8 8.7 0.2];
[path, nodes_expanded] = dijkstrav4(map, start, stop, true);
nodes_expanded
size(path)
%plot_path(map,path);
assertPathValid(testcase, map, path, start, stop, [0 10], [0 10], [0 6]);
end
%}