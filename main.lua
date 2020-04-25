local AStar = require 'astar'

local map = {}
local map_w, map_h = 36, 24
local cached_nodes = {}
local checked_nodes = {}

local lg = love.graphics

-- Node must be able to check if they are the same
-- so the example cannot directly return a different table for same coord
local function get_node(x, y)
  local row = cached_nodes[y]
  if not row then row = {}; cached_nodes[y] = row end
  local node = row[x]
  if not node then node = { x = x, y = y, cost = 0 }; row[x] = node end
  return node
end

local neighbors_offset = {
  { -1, -1 }, { 0, -1 }, { 1, -1 },
  { -1, 0 }, { 1, 0 },
  { -1, 1 }, { 0, 1 }, { 1, 1 },
}
-- Return all neighbor nodes. Means a target that can be moved from the current node
function map:get_neighbors(node)
  local nodes = {}
  local x, y = node.x, node.y
  for i, offset in ipairs(neighbors_offset) do
    local tnode = get_node(x + offset[1], y + offset[2])
    if tnode.cost >= 0 and tnode.x >= 0 and tnode.x < map_w and tnode.y >=0 and tnode.y < map_h then
      nodes[#nodes + 1] = tnode
    end
  end
  return nodes
end

-- Cost of two adjacent nodes
function map:get_cost(from_node, to_node)
  local dx, dy = from_node.x - to_node.x, from_node.y - to_node.y
  return math.sqrt(dx * dx + dy * dy) + (from_node.cost + to_node.cost) * 0.5
end

-- For heuristic. Estimate cost of current node to goal node
function map:estimate_cost(node, goal_node)
  checked_nodes[#checked_nodes + 1] = node
  return self:get_cost(node, goal_node) * 1.5 + (node.cost + goal_node.cost) * 0.5
end

local finder = AStar.new(map)
local path, gscore, hscore
local start_x, start_y = 1, 1
local goal_x, goal_y = 34, 22
local find_time = 0

local function update_path()
  checked_nodes = {}

  local st = love.timer.getTime()
  path, gscore, hscore = finder:find(get_node(start_x, start_y), get_node(goal_x, goal_y))
  find_time = (love.timer.getTime() - st) * 1000

  if path then
    for i, node in ipairs(path) do
      path[node] = true
    end
  end
end

local mcx, mcy = 0, 0

function love.load()
  update_path()
end

function love.update(dt)
  local mx, my = love.mouse.getPosition()
  local w, h = love.graphics.getDimensions()
  local cell_w, cell_h = w / map_w, h / map_h
  mcx, mcy = math.floor(mx / cell_w), math.floor(my / cell_h)
  local changed = false

  local mdown = love.mouse.isDown
  if mdown(1) and start_x ~= mcx and start_y ~= mcy then
    start_x, start_y = mcx, mcy
    changed = true
  end

  if mdown(2) and goal_x ~= mcx and goal_y ~= mcy then
    goal_x, goal_y = mcx, mcy
    changed = true
  end

  local kbdown = love.keyboard.isDown
  local new_cost = nil
  if kbdown('1') then
    new_cost = 0
  elseif kbdown('2') then
    new_cost = 1
  elseif kbdown('3') then
    new_cost = 2
  elseif kbdown('4') then
    new_cost = -1
  end

  if new_cost then
    local node = get_node(mcx, mcy)
    if node.cost ~= new_cost then
      node.cost = new_cost
      changed = true
    end
  end

  if changed then
    update_path()
  end
end

function love.draw()
  local w, h = love.graphics.getDimensions()
  local cell_w, cell_h = w / map_w, h / map_h

  lg.setBackgroundColor(0.5, 0.5, 0.5, 1)
  for i = 0, h - 1 do
    for j = 0, w - 1 do
      local x, y = j * cell_w, i * cell_h
      local node = get_node(j, i)
      local cost = node.cost
      if cost ~= 0 then
        if cost == -1 then
          lg.setColor(0.1, 0.1, 0.1, 1)
        elseif cost == 1 then
          lg.setColor(1, 1, 0.5, 1)
        elseif cost == 2 then
          lg.setColor(0.5, 0.5, 0.1, 1)
        end
        lg.rectangle('fill', x, y, cell_w, cell_h)
      end
    end
  end

  if checked_nodes then
    for _, node in ipairs(checked_nodes) do
      local x, y = node.x * cell_w, node.y * cell_h
      lg.setColor(1, 0, 0, 0.2)
      lg.rectangle('fill', x, y, cell_w, cell_h)
      lg.setColor(0.7, 0.7, 0.9)
      lg.print(string.format('%.1f\n%.1f', gscore[node], hscore[node]), x + 3, y + 3)
    end
  end

  if path then
    lg.setColor(0, 1, 0, 0.4)
    for _, node in ipairs(path) do
      lg.rectangle('fill', node.x * cell_w, node.y * cell_h, cell_w, cell_h)
    end
  end

  lg.setColor(1, 1, 1)
  lg.circle('fill', (start_x + 0.5) * cell_w, (start_y + 0.5) * cell_h, cell_h / 3)
  lg.setColor(0, 0, 1)
  lg.circle('fill', (goal_x + 0.5) * cell_w, (goal_y + 0.5) * cell_h, cell_h / 3)

  lg.setColor(1, 1, 1)
  local str = ''
  str = str..string.format("\n FPS: %i", love.timer.getFPS())

  str = str..'\n'
  local mnode = get_node(mcx, mcy)
  str = str..string.format("\n mouse coord: %i, %i", mcx, mcy)
  str = str..string.format("\n mouse node cost: %i", mnode.cost)

  str = str..'\n'
  str = str..string.format("\n checked nodes: %i", #checked_nodes)
  str = str..string.format("\n time: %.2fms", find_time)

  str = str..'\n'
  str = str..string.format("\n left/right click: move start/goal", find_time)
  str = str..string.format("\n keyboard: 1: cost 0; 2: cost 1; 3 cost 2; 4 blocked", find_time)
  lg.print(str, 10, 10)
end
