/**  <module> refills

  Copyright (C) 2018 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Daniel Beßler
  @license BSD
*/

:- module(refills,
    [
      refills_init_test_shop/0,
      refills_init_iai_shop/0,
      refills_spawn_facings/0
    ]).

:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).

:- use_module(library('shop')).

refills_init_iai_shop :-
  owl_parser:owl_parse('package://knowrob_refills/owl/dm-market-iai.owl', belief_state).

refills_init_test_shop :-
  belief_parse('package://knowrob_refills/owl/shop-test.owl'),
  refills_spawn_facings.

refills_spawn_facings :-
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerStanding'), (
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.0, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.2, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.35, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.6, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.85, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 1.0, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfLabel', 0.1, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfLabel', 0.275, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfLabel', 0.475, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfLabel', 0.725, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfLabel', 0.925, _)
  )).
