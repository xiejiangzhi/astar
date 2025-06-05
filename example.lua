-- require('jit.v').on('jit.trace')

require 'table.new'

local AStar = require 'astar'

local map = {}
local size = 500
local cached_nodes = table.new(size * size, 0)

for x = 1, size do
  for y = 1, size do
    local idx = (y - 1) * size + x
    local s = ((x % 2 == 0) and (y % 2 == 0)) and 10 or 1
    local node = { x = x, y = y, s = s }
    cached_nodes[idx] = node
  end
end

-- Node must be able to check if they are the same
-- Cannot directly return different tables for the same coord
-- The library doesn't change nodes, so you able to reuse your node or create a C struct for faster
local function get_node(x, y)
  local idx = (y - 1) * size + x
  return cached_nodes[idx]
end

local neighbors_offset = {
  { -1, -1 }, { 0, -1 }, { 1, -1 },
  { -1, 0 }, { 1, 0 },
  { -1, 1 }, { 0, 1 }, { 1, 1 },
}
-- Return all neighbor nodes. Means a target that can be moved from the current node
function map:get_neighbors(node, from_node, add_neighbor_fn, userdata)
  local nodes = {}
  for i, offset in ipairs(neighbors_offset) do
    local nx = node.x + offset[1]
    local ny = node.y + offset[2]
    if nx % 5 ~= 0 or ny % 5 ~= 0 then
      local neighbor = get_node(nx, ny)
      if neighbor then
        add_neighbor_fn(neighbor)
      end
    end
  end
  return nodes
end

-- Cost of two adjacent nodes.
-- Distance, distance + cost or other comparison value you want
function map:get_cost(from_node, to_node)
  local v = math.sqrt((from_node.x - to_node.x)^2 + (from_node.y - to_node.y)^2)
  return v * (from_node.s + to_node.s) * 0.5
end

-- For heuristic. Estimate cost of current node to goal node
-- As close to the real cost as possible
function map:estimate_cost(node, goal_node)
  return math.sqrt((node.x - goal_node.x)^2 + (node.y - goal_node.y)^2)
end

local finder = AStar.new(map)


local function test(name, sx, sy, tx, ty, print_path)
  print(string.format("%s %i,%i -> %i,%i", name, sx, sy, tx, ty))
  for i = 1, 3 do
    local start, goal = get_node(sx, sy), get_node(tx, ty)
    local st = os.clock()
    local path, g_score = finder:find(start, goal)
    local cost = (os.clock() - st) * 1000
    local tnodes = 0
    for n, v in pairs(g_score) do
      tnodes = tnodes + 1
    end

    if path then
      print(string.format('cost: %9.4fms, test valid nodes: %i, path nodes: %s', cost, tnodes, #path))
      if print_path then
        local r = {}
        for _, node in ipairs(path) do
          r[#r + 1] = node.x..','..node.y
        end
        print(table.concat(r, ' > '))
      end
    else
      print(string.format("cost: %.4fms, tested valid nodes: %i, Not found path", cost, tnodes))
    end
  end
end

print("map size: "..size..'x'..size)

test('a', 1, 1, size - 1, size - 1)
test('b', 1, 1, 201, 201)
test('b', 1, 1, size - 1, math.floor(size * 0.5))

test('c', 1, 1, size, size)
