/*
The MIT License (MIT)

Copyright (c) 2015 Mathias Westerdahl

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "usv_map/jc_voronoi.h"

void jcv_diagram_free( jcv_diagram* d )
{
    jcv_context_internal* internal = d->internal;
    void* memctx = internal->memctx;
    FJCVFreeFn freefn = internal->free;
    while(internal->memblocks)
    {
        jcv_memoryblock* p = internal->memblocks;
        internal->memblocks = internal->memblocks->next;
        freefn( memctx, p );
    }

    freefn( memctx, internal->mem );
}

const jcv_site* jcv_diagram_get_sites( const jcv_diagram* diagram )
{
    return diagram->internal->sites;
}

const jcv_edge* jcv_diagram_get_edges( const jcv_diagram* diagram )
{
    jcv_edge e;
    e.next = diagram->internal->edges;
    return jcv_diagram_get_next_edge(&e);
}

const jcv_edge* jcv_diagram_get_next_edge( const jcv_edge* edge )
{
    const jcv_edge* e = edge->next;
    while (e != 0 && jcv_point_eq(&e->pos[0], &e->pos[1])) {
        e = e->next;
    }
    return e;
}

// CLIPPING
int jcv_boxshape_test(const jcv_clipper* clipper, const jcv_point p)
{
    return p.x >= clipper->min.x && p.x <= clipper->max.x &&
           p.y >= clipper->min.y && p.y <= clipper->max.y;
}

// The line equation: ax + by + c = 0
// see jcv_edge_create
int jcv_boxshape_clip(const jcv_clipper* clipper, jcv_edge* e)
{
    jcv_real pxmin = clipper->min.x;
    jcv_real pxmax = clipper->max.x;
    jcv_real pymin = clipper->min.y;
    jcv_real pymax = clipper->max.y;

    jcv_real x1, y1, x2, y2;
    jcv_point* s1;
    jcv_point* s2;
    if (e->a == (jcv_real)1 && e->b >= (jcv_real)0)
    {
        s1 = jcv_is_valid(&e->pos[1]) ? &e->pos[1] : 0;
        s2 = jcv_is_valid(&e->pos[0]) ? &e->pos[0] : 0;
    }
    else
    {
        s1 = jcv_is_valid(&e->pos[0]) ? &e->pos[0] : 0;
        s2 = jcv_is_valid(&e->pos[1]) ? &e->pos[1] : 0;
    };

    if (e->a == (jcv_real)1) // delta x is larger
    {
        y1 = pymin;
        if (s1 != 0 && s1->y > pymin)
        {
            y1 = s1->y;
        }
        if( y1 > pymax )
        {
            y1 = pymax;
        }
        x1 = e->c - e->b * y1;
        y2 = pymax;
        if (s2 != 0 && s2->y < pymax)
            y2 = s2->y;

        if( y2 < pymin )
        {
            y2 = pymin;
        }
        x2 = (e->c) - (e->b) * y2;
        // Never occurs according to lcov
        // if( ((x1 > pxmax) & (x2 > pxmax)) | ((x1 < pxmin) & (x2 < pxmin)) )
        // {
        //     return 0;
        // }
        if (x1 > pxmax)
        {
            x1 = pxmax;
            y1 = (e->c - x1) / e->b;
        }
        else if (x1 < pxmin)
        {
            x1 = pxmin;
            y1 = (e->c - x1) / e->b;
        }
        if (x2 > pxmax)
        {
            x2 = pxmax;
            y2 = (e->c - x2) / e->b;
        }
        else if (x2 < pxmin)
        {
            x2 = pxmin;
            y2 = (e->c - x2) / e->b;
        }
    }
    else // delta y is larger
    {
        x1 = pxmin;
        if( s1 != 0 && s1->x > pxmin )
            x1 = s1->x;
        if( x1 > pxmax )
        {
            x1 = pxmax;
        }
        y1 = e->c - e->a * x1;
        x2 = pxmax;
        if( s2 != 0 && s2->x < pxmax )
            x2 = s2->x;
        if( x2 < pxmin )
        {
            x2 = pxmin;
        }
        y2 = e->c - e->a * x2;
        // Never occurs according to lcov
        // if( ((y1 > pymax) & (y2 > pymax)) | ((y1 < pymin) & (y2 < pymin)) )
        // {
        //     return 0;
        // }
        if( y1 > pymax )
        {
            y1 = pymax;
            x1 = (e->c - y1) / e->a;
        }
        else if( y1 < pymin )
        {
            y1 = pymin;
            x1 = (e->c - y1) / e->a;
        }
        if( y2 > pymax )
        {
            y2 = pymax;
            x2 = (e->c - y2) / e->a;
        }
        else if( y2 < pymin )
        {
            y2 = pymin;
            x2 = (e->c - y2) / e->a;
        };
    };

    e->pos[0].x = x1;
    e->pos[0].y = y1;
    e->pos[1].x = x2;
    e->pos[1].y = y2;

    // If the two points are equal, the result is invalid
    return (x1 == x2 && y1 == y2) ? 0 : 1;
}

void jcv_boxshape_fillgaps(const jcv_clipper* clipper, jcv_context_internal* allocator, jcv_site* site)
{
    // They're sorted CCW, so if the current->pos[1] != next->pos[0], then we have a gap
    jcv_graphedge* current = site->edges;
    if( !current )
    {
        // No edges, then it should be a single cell
        assert( allocator->numsites == 1 );

        jcv_graphedge* gap = jcv_alloc_graphedge(allocator);
        gap->neighbor   = 0;
        gap->pos[0]     = clipper->min;
        gap->pos[1].x   = clipper->max.x;
        gap->pos[1].y   = clipper->min.y;
        gap->angle      = jcv_calc_sort_metric(site, gap);
        gap->next       = 0;
        gap->edge       = jcv_create_gap_edge(allocator, site, gap);

        current = gap;
        site->edges = gap;
    }

    jcv_graphedge* next = current->next;
    if( !next )
    {
        // Only one edge, then we assume it's a corner gap
        jcv_graphedge* gap = jcv_alloc_graphedge(allocator);
        jcv_create_corner_edge(allocator, site, current, gap);
        gap->edge = jcv_create_gap_edge(allocator, site, gap);

        gap->next = current->next;
        current->next = gap;
        current = gap;
        next = site->edges;
    }

    while( current && next )
    {
        if( jcv_point_on_box_edge(&current->pos[1], &clipper->min, &clipper->max) && !jcv_point_eq(&current->pos[1], &next->pos[0]) )
        {
            // Border gap
            if( current->pos[1].x == next->pos[0].x || current->pos[1].y == next->pos[0].y)
            {
                jcv_graphedge* gap = jcv_alloc_graphedge(allocator);
                gap->neighbor   = 0;
                gap->pos[0]     = current->pos[1];
                gap->pos[1]     = next->pos[0];
                gap->angle      = jcv_calc_sort_metric(site, gap);
                gap->edge       = jcv_create_gap_edge(allocator, site, gap);

                gap->next = current->next;
                current->next = gap;
            }
            else if( jcv_point_on_box_edge(&current->pos[1], &clipper->min, &clipper->max) &&
                     jcv_point_on_box_edge(&next->pos[0], &clipper->min, &clipper->max) )
            {
                jcv_graphedge* gap = jcv_alloc_graphedge(allocator);
                jcv_create_corner_edge(allocator, site, current, gap);
                gap->edge = jcv_create_gap_edge(allocator, site, gap);
                gap->next = current->next;
                current->next = gap;
            }
            else
            {
                // something went wrong, abort instead of looping indefinitely
                break;
            }
        }

        current = current->next;
        if( current )
        {
            next = current->next;
            if( !next )
                next = site->edges;
        }
    }
}

void jcv_diagram_generate( int num_points, const jcv_point* points, const jcv_rect* rect, const jcv_clipper* clipper, jcv_diagram* d )
{
    jcv_diagram_generate_useralloc(num_points, points, rect, clipper, 0, jcv_alloc_fn, jcv_free_fn, d);
}

void jcv_diagram_generate_useralloc(int num_points, const jcv_point* points, const jcv_rect* rect, const jcv_clipper* clipper, void* userallocctx, FJCVAllocFn allocfn, FJCVFreeFn freefn, jcv_diagram* d)
{
    if( d->internal )
        jcv_diagram_free( d );

    jcv_context_internal* internal = jcv_alloc_internal(num_points, userallocctx, allocfn, freefn);

    internal->beachline_start = jcv_halfedge_new(internal, 0, 0);
    internal->beachline_end = jcv_halfedge_new(internal, 0, 0);

    internal->beachline_start->left     = 0;
    internal->beachline_start->right    = internal->beachline_end;
    internal->beachline_end->left       = internal->beachline_start;
    internal->beachline_end->right      = 0;

    internal->last_inserted = 0;

    int max_num_events = num_points*2; // beachline can have max 2*n-5 parabolas
    jcv_pq_create(internal->eventqueue, max_num_events, (void**)internal->eventmem);

    internal->numsites = num_points;
    jcv_site* sites = internal->sites;

    for( int i = 0; i < num_points; ++i )
    {
        sites[i].p        = points[i];
        sites[i].edges    = 0;
        sites[i].index    = i;
    }

    qsort(sites, (size_t)num_points, sizeof(jcv_site), jcv_point_cmp);

    jcv_clipper box_clipper;
    if (clipper == 0) {
        box_clipper.test_fn = jcv_boxshape_test;
        box_clipper.clip_fn = jcv_boxshape_clip;
        box_clipper.fill_fn = jcv_boxshape_fillgaps;
        clipper = &box_clipper;
    }
    internal->clipper = *clipper;

    jcv_rect tmp_rect;
    tmp_rect.min.x = tmp_rect.min.y = JCV_FLT_MAX;
    tmp_rect.max.x = tmp_rect.max.y = -JCV_FLT_MAX;
    jcv_prune_duplicates(internal, &tmp_rect);

    // Prune using the test second
    if (internal->clipper.test_fn)
    {
        // e.g. used by the box clipper in the test_fn
        internal->clipper.min = rect ? rect->min : tmp_rect.min;
        internal->clipper.max = rect ? rect->max : tmp_rect.max;

        jcv_prune_not_in_shape(internal, &tmp_rect);

        // The pruning might have made the bounding box smaller
        if (!rect) {
            // In the case of all sites being all on a horizontal or vertical line, the
            // rect area will be zero, and the diagram generation will most likely fail
            jcv_rect_round(&tmp_rect);
            jcv_rect_inflate(&tmp_rect, 10);

            internal->clipper.min = tmp_rect.min;
            internal->clipper.max = tmp_rect.max;
        }
    }

    internal->rect = rect ? *rect : tmp_rect;

    d->min      = internal->rect.min;
    d->max      = internal->rect.max;
    d->numsites = internal->numsites;
    d->internal = internal;

    internal->bottomsite = jcv_nextsite(internal);

    jcv_priorityqueue* pq = internal->eventqueue;
    jcv_site* site = jcv_nextsite(internal);

    int finished = 0;
    while( !finished )
    {
        jcv_point lowest_pq_point;
        if( !jcv_pq_empty(pq) )
        {
            jcv_halfedge* he = (jcv_halfedge*)jcv_pq_top(pq);
            lowest_pq_point.x = he->vertex.x;
            lowest_pq_point.y = he->y;
        }

        if( site != 0 && (jcv_pq_empty(pq) || jcv_point_less(&site->p, &lowest_pq_point) ) )
        {
            jcv_site_event(internal, site);
            site = jcv_nextsite(internal);
        }
        else if( !jcv_pq_empty(pq) )
        {
            jcv_circle_event(internal);
        }
        else
        {
            finished = 1;
        }
    }

    for( jcv_halfedge* he = internal->beachline_start->right; he != internal->beachline_end; he = he->right )
    {
        jcv_finishline(internal, he->edge);
    }

    jcv_fillgaps(d);
}