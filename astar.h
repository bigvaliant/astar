/*	
    Copyright (C) <2012>  <huangweilook@21cn.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef _ASTAR_H
#define _ASTAR_H
#include <stdlib.h>
#include <map>
#include "minheap.h"
#include <list>
#include <vector>

extern int direction[8][2];

class AStar{
public:
	class dlist;
	struct dnode{
		dnode():next(NULL),pre(NULL),_dlist(NULL){}
		dnode *next;
		dnode *pre;
		dlist *_dlist;
	};

	class dlist{
		public:
			dlist():size(0){
				head.next = &tail;
				tail.pre = &head;
			}
			size_t Size(){
				return size;
			}

			bool Empty(){
				return size == 0;
			}

			dnode *Begin(){
				if(Empty())return &tail;
				return head.next;
			}

			dnode *End(){
				return &tail;
			}

			void Push(dnode *n){
				if(n->_dlist || n->next || n->pre) return;
				tail.pre->next = n;
				n->pre = tail.pre;
				tail.pre = n;
				n->_dlist = this;
				n->next = &tail;
				++size;
			}

			void Remove(dnode *n)
			{
				if(n->_dlist != this || (!n->pre && !n->next))
					return;
				n->pre->next = n->next;
				n->next->pre = n->pre;
				n->pre = n->next = NULL;
				n->_dlist = NULL;
				--size;
			}

			dnode* Pop(){
				if(Empty())
					return NULL;
				else
				{
					dnode *n = head.next;
					Remove(n);
					return n;
				}
			}

		private:
			dnode head;
			dnode tail;
			size_t size;
	};

	struct mapnode : public heapele,public dnode
	{
		mapnode *parent;
		double G;//从初始点到当前点的开销
		double H;//从当前点到目标点的估计开销
		double F;
		int    x;
		int    y;
		int    value;
	};

private:

	static bool _less(heapele*l,heapele*r)
	{
		return ((mapnode*)l)->F < ((mapnode*)r)->F;
	}

	static void _clear(heapele*e){
		((mapnode*)e)->F = ((mapnode*)e)->G = ((mapnode*)e)->H = 0;
	}

	mapnode *get_mapnode(int x,int y)
	{
		if(x < 0 || x >= m_xcount || y < 0 || y >= m_ycount)
			return NULL;
		return &m_map[y*m_xcount+x];
	}

	//获得当前maze_node的8个临近节点,如果是阻挡点会被忽略
	std::vector<mapnode*>* get_neighbors(mapnode *mnode)
	{
		m_neighbors.clear();
		int i = 0;
		int c = 0;
		for( ; i < 8; ++i)
		{
			int x = mnode->x + direction[i][0];
			int y = mnode->y + direction[i][1];
			mapnode *tmp = get_mapnode(x,y);
			if(tmp){
				if(tmp->value != 0xFFFFFFFF)
					m_neighbors.push_back(tmp);
			}
		}
		if(m_neighbors.empty()) return NULL;
		else return &m_neighbors;
	}

	//计算到达相临节点需要的代价
	double cost_2_neighbor(mapnode *from,mapnode *to)
	{
		int delta_x = from->x - to->x;
		int delta_y = from->y - to->y;
		int i = 0;
		for( ; i < 8; ++i)
		{
			if(direction[i][0] == delta_x && direction[i][1] == delta_y)
				break;
		}
		if(i < 4)
			return 50.0f;
		else if(i < 8)
			return 60.0f;
		else
		{
			assert(0);
			return 0.0f;
		}	
	}

	double cost_2_goal(mapnode *from,mapnode *to)
	{
		int delta_x = abs(from->x - to->x);
		int delta_y = abs(from->y - to->y);
		return (delta_x * 50.0f) + (delta_y * 50.0f);
	}

	void   reset();

	std::vector<mapnode*> m_neighbors;//中间计算用

	mapnode *m_map;
	int      m_xcount;
	int      m_ycount;
	minheap  open_list;
	dlist    close_list;
public:
	AStar():open_list(8192,_less,_clear){} //初始化astar

	bool Init(int x,int y,std::map<std::pair<int,int>,int> &values);

	~AStar(){
		if(m_map) free(m_map);
	}

	bool find_path(int x,int y,int x1,int y1,std::list<mapnode*> &path);

};

#endif
