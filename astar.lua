-- AStar
--
-- map:
--  get_neighbors(node, from_node, add_neighbor_fn, userdata) -- all moveable neighbors
--  get_cost(from_node, to_node, userdata)
--  estimate_cost(start_node, goal_node, userdata)
--
-- node:
--  x:
--  y:
--  ==: check two node is same
--

local M = {}
M.__index = M

local BinaryHeap = require('binaryheap')

function M.new(...)
  local obj = setmetatable({}, M)
  obj:init(...)
  return obj
end

function M:init(map)
  assert(
    map.get_neighbors and map.get_cost and map.estimate_cost,
    "Invalid map, must include get_neighbors, get_cost and estimate_cost functions"
  )
  self.map = map
end

-- start: start node
-- goal: goal node
-- return path_or_nil, g_score, h_score, f_score, came_from_map
function M:find(start, goal, userdata)
  local map = self.map

  -- local total_nodes = map.total_nodes
	local closedset = {}
	local came_from = {}

	local g_score, h_score, f_score = {}, {}, {}
	g_score[start] = 0
  h_score[start] = map:estimate_cost(start, goal, userdata)
	f_score[start] = h_score[start]

	local openmap = { [start] = true }
  local openlist = BinaryHeap.minHeap(function(a, b)
    return f_score[a] < f_score[b]
  end)
  openlist:insert(start)
	local current

	local add_neighbor_fn = function(neighbor, cost)
		if not closedset[neighbor] then
			if not cost then cost = map:get_cost(current, neighbor, userdata) end
			local tentative_g_score = g_score[current] + cost

			if not openmap[neighbor] or tentative_g_score < g_score[neighbor] then
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g_score
				h_score[neighbor] = h_score[neighbor] or map:estimate_cost(neighbor, goal, userdata)
				f_score[neighbor] = tentative_g_score + h_score[neighbor]

        openlist:insert(neighbor)
				openmap[neighbor] = true
			end
		end
	end

	while true do
		current = openlist:pop()
    if not current then
      break
    end
    -- openlist[#openlist] = nil
    openmap[current] = nil

		if current == goal then
			local path = M._unwind_path({}, came_from, goal)
			table.insert(path, goal)
			return path, g_score, h_score, f_score, came_from
		end
		closedset[current] = true

    local from_node = came_from[current]
		map:get_neighbors(current, from_node, add_neighbor_fn, userdata)
	end

	return nil, g_score, h_score, f_score, came_from -- no valid path
end

----------------------------

function M._unwind_path(flat_path, map, current_node)
	if map[current_node] then
		table.insert(flat_path, 1, map [ current_node ])
		return M._unwind_path(flat_path, map, map [ current_node ])
	else
		return flat_path
	end
end

return M
