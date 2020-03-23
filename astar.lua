-- AStar
--
-- map:
--  get_node(x, y)
--  get_neighbors(x, y) -- all moveable neighbors
--  get_cost(from_node, to_node)
--  estimate_cost(from_node, to_node)
--
-- node:
--  x:
--  y:
--  ==: check two node is same

local M = {}
M.__index = M
local private = {}

function M.new(...)
  local obj = setmetatable({}, M)
  obj:init(...)
  return obj
end

function M:init(map)
  self.map = map
  assert(
    map.get_node and map.get_neighbors and map.get_cost and map.estimate_cost,
    "Invalid map, must include get_node, get_neighbors, get_cost and estimate_cost functions"
  )
end

function M:find(start_x, start_y, goal_x, goal_y)
  local map = self.map
  local start = map:get_node(start_x, start_y)
  local goal = map:get_node(goal_x, goal_y)

	local openset = { start, [start] = true }
	local closedset = {}
	local came_from = {}

	local g_score, h_score, f_score = {}, {}, {}
	g_score[start] = 0
  h_score[start] = map:estimate_cost(start, goal)
	f_score[start] = h_score[start]

  local compare_fn = function(a, b)
    return f_score[a] > f_score[b]
  end

	while #openset > 0 do
		local current = openset[#openset]
		openset[#openset] = nil
    openset[current] = nil

		if current == goal then
			local path = private.unwind_path({}, came_from, goal)
			table.insert(path, goal)
			return path
		end
		closedset[current] = true

		local neighbors = map:get_neighbors(current)
		for _, neighbor in ipairs (neighbors) do
			if not closedset[neighbor] then
				local tentative_g_score = g_score[current] + map:get_cost(current, neighbor)

        local in_openset = openset[neighbor]
				if not in_openset or tentative_g_score < g_score[neighbor] then
					came_from[neighbor] = current
					g_score[neighbor] = tentative_g_score
          local h = h_score[neighbor]
          if not h then
            h = map:estimate_cost(neighbor, goal)
            h_score[neighbor] = h
          end
					f_score[neighbor] = tentative_g_score + h

					if not in_openset then
						private.binsert(openset, neighbor, compare_fn)
            openset[neighbor] = true
					end
				end
			end
		end
	end

	return nil -- no valid path
end

----------------------------

function private.unwind_path(flat_path, map, current_node)
	if map[current_node] then
		table.insert(flat_path, 1, map [ current_node ])
		return private.unwind_path(flat_path, map, map [ current_node ])
	else
		return flat_path
	end
end

-- binary insert to a sorted table
-- http://lua-users.org/wiki/BinaryInsert
function private.binsert(t, value, compare_fn)
  --  Initialise numbers
  local i_start, i_end, i_mid, i_state = 1, #t, 1, 0
  local v1, v2, cr
  -- Get insert position
  while i_start <= i_end do
    -- calculate middle
    i_mid = math.floor( (i_start + i_end ) / 2 )
    v1, v2 = value, t[i_mid]
    if compare_fn then
      cr = compare_fn(v1, v2)
    else
      cr = v1 < v2
    end

    if cr then
      i_end, i_state = i_mid - 1, 0
    else
      i_start, i_state = i_mid + 1, 1
    end
  end

  table.insert(t, i_mid + i_state, value)
end

return M
