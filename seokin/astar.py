import json
import random
import math
import numpy as np
import matplotlib.pyplot as plt

class Astar():
    def __init__(self, json_file='./utils/airsim_nh.json'):
        with open(json_file) as f:
            self.nodes = json.load(f)['nodes']

        self.road_shift = 2
        self.get_random_start_end_points()

    def cal_x_y(self, _node_1, _node_2):
        try:
            _pos_x = int(random.uniform(_node_1['pos_x'], _node_2['pos_x']))
        except:
            _pos_x = _node_1['pos_x']

        try:
            _pos_y = int(random.uniform(_node_1['pos_y'], _node_2['pos_y']))
        except:
            _pos_y = _node_1['pos_y']

        return _pos_x, _pos_y

    def get_random_start_end_points(self, start=True):
        if start == True:
            _node_1 = random.choice(self.nodes)
            _conn_node = random.choice(_node_1['connected_nodes'])
            _node_2 = next(node for node in self.nodes if node["name"] == _conn_node)

            _pos_x_s, _pos_y_s = self.cal_x_y(_node_1, _node_2)

            self.start_point = {
                'name': 0,
                'pos_x': _pos_x_s,
                'pos_y': _pos_y_s,
                'connected_nodes': [_node_1['name'], _node_2['name']],
                'F':0,
                'G':0,
                'H':0,
                'parent_node':None
            }
        else:
            self.start_point = self.end_point
            self.start_point['name'] = 0
            self.start_point['F'] = 0
            self.start_point['G'] = 0
            self.start_point['H'] = 0
            self.start_point['parent_node'] = None

            _node_1 = next(node for node in self.nodes if node["name"] == self.start_point['connected_nodes'][0])
            _node_2 = next(node for node in self.nodes if node["name"] == self.start_point['connected_nodes'][1])

        # self.close_list = []

        while True:
            _node_3 = random.choice(self.nodes)
            _conn_node = random.choice(_node_3['connected_nodes'])
            _node_4 = next(node for node in self.nodes if node["name"] == _conn_node)

            _pos_x_e, _pos_y_e = self.cal_x_y(_node_3, _node_4)

            if not _node_3['name'] in [_node_1['name'], _node_2['name']] and not _node_3['name'] in [_node_1['name'], _node_2['name']]:
                self.end_point = {
                    'name': -1,
                    'pos_x': _pos_x_e,
                    'pos_y': _pos_y_e,
                    'connected_nodes': [_node_3['name'], _node_4['name']]
                }

                for i, n in enumerate(self.nodes):
                    if n['name'] == _node_3['name'] or n['name'] == _node_4['name']:
                        n['connected_nodes'].append(-1)
                        self.nodes[i] = n

                break

        # Calculate F,G,H scores
        _node_1 = self.cal_f_g_h(self.start_point, _node_1, self.end_point)
        _node_2 = self.cal_f_g_h(self.start_point, _node_2, self.end_point)

        self.open_list = [_node_1, _node_2]
        self.close_list = [self.start_point]
        
    def cal_cost(self, s, n):
        return math.sqrt((s['pos_x'] - n['pos_x']) ** 2 + (s['pos_y'] - n['pos_y']) ** 2)

    def cal_f_g_h(self, s, n, e):
        g = self.cal_cost(s, n)
        h = self.cal_cost(n, e)

        if 'G' not in n or n['G'] > g:
            n['G'] = g
            n['H'] = h
            n['F'] = g+h
            n['parent_node'] = s['name']

        return n

    def compute(self, path_continue=False):
        if path_continue == True:
            self.get_random_start_end_points(start=False)
            
        while True:
            while True:
                min_node = min(self.open_list, key=lambda x:x['F'])
                self.close_list.append(min_node)

                if min_node['name'] == -1:
                    # print("Terminate")
                    
                    _cur_node = -1
                    self.node_routes = [next(node for node in self.close_list if node["name"] == _cur_node)]
                    while True:
                        if _cur_node == 0:
                            break
                        parent_node = next(node for node in self.close_list if node["name"] == _cur_node)['parent_node']
                        self.node_routes.append(next(node for node in self.close_list if node["name"] == parent_node))
                        _cur_node = parent_node

                    break
                
                self.open_list[:] = [d for d in self.open_list if d.get('name') != min_node['name']]          

                target_nodes = self.get_unique_nodes(min_node['connected_nodes'], self.close_list)
                
                for i, n in enumerate(target_nodes):
                    target_nodes[i] = self.cal_f_g_h(min_node, n, self.end_point)

                self.open_list += target_nodes

            self.get_path_coords()
            self.get_corner_path()

            _s_point = self.coords[0]

            # When car is located near the corner, compute again.
            _start_direction = self.get_start_direction(self.coords)
            self.coords.pop(0)

            if _start_direction is not None:
                break    
        
        return _s_point, _start_direction, self.coords
            

    def get_start_direction(self, coordinates):
        start_direction_x = coordinates[1][0] - coordinates[0][0]
        start_direction_y = coordinates[1][1] - coordinates[0][1]
        # print(start_direction_x, start_direction_y, coordinates[0][0],coordinates[0][1])
        
        start_direction = None
    
        if (start_direction_x < 0 and start_direction_y == 0):
            start_direction = math.radians(180)
        elif (start_direction_x > 0 and start_direction_y == 0):
            start_direction = math.radians(0)
        #elif (start_direction_x == 0 and start_direction_y > 0):
        #    start_direction = math.radians(90)
        #elif (start_direction_x == 0 and start_direction_y < 0):
        #    start_direction = math.radians(-90)

        return start_direction

    def get_path_coords(self):
        self.straight_coords = []
        ref_x = [-128, -128, 128, 128]
        ref_y = [-128, 128, -128, 128]
        xs = []
        ys = []
        for n in self.node_routes:
            if n['name'] != 0:
                pn = next(node for node in self.node_routes if node["name"] == n['parent_node'])
                if n['pos_x'] == pn['pos_x']:
                    if n['pos_y'] <= pn['pos_y']:
                        for y in range(n['pos_y'], pn['pos_y']):
                            # print(n['name'],'y:', y)
                            _coord = (n['pos_x']-self.road_shift, y)
                            self.straight_coords.append(_coord)
                            xs.append(n['pos_x']-self.road_shift)
                            ys.append(y)
                    else:
                        for y in range(n['pos_y'], pn['pos_y'], -1):
                            # print(n['name'],'y:', y)
                            _coord = (n['pos_x']+self.road_shift, y)
                            self.straight_coords.append(_coord)
                            xs.append(n['pos_x']+self.road_shift)
                            ys.append(y)
                elif n['pos_y'] == pn['pos_y']:
                    if n['pos_x'] <= pn['pos_x']:
                        for x in range(n['pos_x'], pn['pos_x']):
                            # print(n['name'],'x:', x)
                            _coord = (x, n['pos_y']+self.road_shift)
                            self.straight_coords.append(_coord)
                            xs.append(x)
                            ys.append(n['pos_y']+self.road_shift)
                    else:
                        for x in range(n['pos_x'], pn['pos_x'], -1):
                            # print(n['name'],'x:', x)
                            _coord = (x, n['pos_y']-self.road_shift)
                            self.straight_coords.append(_coord)
                            xs.append(x)
                            ys.append(n['pos_y']-self.road_shift)
                
        
        plt.scatter(ref_x, ref_y)

    def cal_vector(self):
        pass

    def get_corner_path(self):
        self.coords = []
        rm_coords = []
        for i, _coord in enumerate(self.straight_coords):
            if i>0 and i<len(self.straight_coords)-1:
                _x = _coord[0]
                _y = _coord[1]

                _prev_coords = self.straight_coords[i-1]
                _prev_x = _prev_coords[0]
                _prev_y = _prev_coords[1]

                _next_coords = self.straight_coords[i+1]
                _next_x = _next_coords[0]
                _next_y = _next_coords[1]

                v1_x = _x - _prev_x
                v1_y = _y - _prev_y

                v2_x = _next_x - _x
                v2_y = _next_y - _y

                size_v1 = math.sqrt(v1_x ** 2 + v1_y ** 2)
                size_v2 = math.sqrt(v2_x ** 2 + v2_y ** 2)

                if (v1_x * v2_x + v1_y * v2_y)/(size_v1*size_v2) > 0 and (v1_x * v2_x + v1_y * v2_y)/(size_v1*size_v2) < 1:
                    # Left turn
                    
                    self.coords.append(_coord)
                    if v1_x == 0 or v1_y == 0:
                        # print("Left Turn")
                        if v2_x > 0 and v2_y > 0:
                            _center_x = _next_x
                            _center_y = _y

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x-(self.road_shift*math.cos(math.radians(degree_intv)))
                                _circle_y = _center_y+(self.road_shift*math.sin(math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                self.coords.append((_circle_x, _circle_y))
                        elif v2_x > 0 and v2_y < 0:
                            _center_x = _x
                            _center_y = _next_y

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x+(self.road_shift*math.cos(math.pi*3/2 + math.radians(degree_intv)))
                                _circle_y = _center_y-(self.road_shift*math.sin(math.pi*3/2 + math.radians(degree_intv)))

                                self.coords.append((_circle_x, _circle_y))
                                
                        elif v2_x < 0 and v2_y > 0:
                            _center_x = _x
                            _center_y = _next_y

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x+(self.road_shift*math.cos(math.pi/2 + math.radians(degree_intv)))
                                _circle_y = _center_y-(self.road_shift*math.sin(math.pi/2 + math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                self.coords.append((_circle_x, _circle_y))
                        elif v2_x < 0 and v2_y < 0:
                            _center_x = _next_x
                            _center_y = _y

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x-(self.road_shift*math.cos(math.pi + math.radians(degree_intv)))
                                _circle_y = _center_y+(self.road_shift*math.sin(math.pi + math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                self.coords.append((_circle_x, _circle_y))
                            
                        # print("LT", _x, _y, _next_coords, v2_x, v2_y)
                        

                        
                elif (v1_x * v2_x + v1_y * v2_y)/(size_v1*size_v2) < 0:
                    # Right turn
                    self.coords.append(_coord)
                    if v1_x == 0 or v1_y == 0:
                        rm_coords.append(_coord)
                        # print("RT", _x, _y, _next_coords, v2_x, v2_x)

                else:
                    self.coords.append(_coord)
        # print(rm_coords)
        rm_index = []
        rt_corner_idx = []
        rt_corner_start_idx = []
        rt_corner_coords = []
        if len(rm_coords) > 0:
            for rm_c in rm_coords:
                for i, _c in enumerate(self.coords):
                    if rm_c == _c:
                        _x = _c[0]
                        _y = _c[1]

                        _prev_coords = self.coords[i-1]
                        _prev_x = _prev_coords[0]
                        _prev_y = _prev_coords[1]

                        _next_coords = self.coords[i+1]
                        _next_x = _next_coords[0]
                        _next_y = _next_coords[1]

                        v1_x = _x - _prev_x
                        v1_y = _y - _prev_y

                        v2_x = _next_x - _x
                        v2_y = _next_y - _y

                        temp_circle_coords = []
                        if v2_x > 0 and v2_y > 0:
                            _center_x = _next_x + 1*self.road_shift
                            _center_y = _y - 1*self.road_shift

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x-(self.road_shift*math.cos(math.radians(degree_intv)))
                                _circle_y = _center_y+(self.road_shift*math.sin(math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                temp_circle_coords.append((_circle_x, _circle_y))
                                
                        elif v2_x > 0 and v2_y < 0:
                            _center_x = _x - 1*self.road_shift
                            _center_y = _next_y - 1*self.road_shift

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x+(self.road_shift*math.cos(math.pi*3/2 + math.radians(degree_intv)))
                                _circle_y = _center_y-(self.road_shift*math.sin(math.pi*3/2 + math.radians(degree_intv)))

                                temp_circle_coords.append((_circle_x, _circle_y))
                                
                        elif v2_x < 0 and v2_y > 0:
                            _center_x = _x + 1*self.road_shift
                            _center_y = _next_y + 1*self.road_shift

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x+(self.road_shift*math.cos(math.pi/2 + math.radians(degree_intv)))
                                _circle_y = _center_y-(self.road_shift*math.sin(math.pi/2 + math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                temp_circle_coords.append((_circle_x, _circle_y))
                        elif v2_x < 0 and v2_y < 0:
                            _center_x = _next_x - 1*self.road_shift
                            _center_y = _y + 1*self.road_shift

                            for degree_intv in range(15,90,15):
                            
                                _circle_x = _center_x-(self.road_shift*math.cos(math.pi + math.radians(degree_intv)))
                                _circle_y = _center_y+(self.road_shift*math.sin(math.pi + math.radians(degree_intv)))
                                # print(degree_intv,(_circle_x, _circle_y))
                                temp_circle_coords.append((_circle_x, _circle_y))

                        rt_corner_idx.append(len(temp_circle_coords))
                        rt_corner_start_idx.append(i-1*self.road_shift)
                        for _c in temp_circle_coords:
                            rt_corner_coords.append(_c)

                        for i_ in range(-(self.road_shift)*2,(self.road_shift+1)):
                            rm_index.append(i+i_+1*self.road_shift)
                        # print("rm_index", rm_index)

            # rm_index = []
            

            # rm_index=[]
            self.coords = [v for i, v in enumerate(self.coords) if i not in rm_index]
                # pass
            _rc_cnt = 0
            for _i, ridx in enumerate(rt_corner_idx):
                for _coord_index in range(ridx):
                    self.coords.insert(rt_corner_start_idx[_i], rt_corner_coords[_rc_cnt])

                    _rc_cnt += 1

            # print(self.coords)

        # plt.scatter([c[0] for c in self.coords], [c[1] for c in self.coords])
        # plt.show()
                
    def get_unique_nodes(self, name_list_1, list_2):
        name_list_2 = []
        for n in list_2:
            name_list_2.append(n['name'])

        r = []
        for name in name_list_1:
            if name not in name_list_2:
                r.append(name)

        result_nodes = []
        for name in r:
            if name != -1:
                result_nodes.append(next(node for node in self.nodes if node["name"] == name))
            else:
                result_nodes.append(self.end_point)
        return result_nodes
        
        

if __name__ == "__main__":
    astar = Astar()
    start_point, end_point, coordinates = astar.compute()
    print ("start", start_point)
    print ("coordinates", coordinates)
    print ("end", end_point)
    