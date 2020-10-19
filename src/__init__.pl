/*
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

:- register_ros_package(knowrob).
% :- register_ros_package(knowrob_objects).
% :- register_ros_package(knowrob_memory).
:- register_ros_package(knowrob_refills).

:- use_module(library('shop')).
:- use_module(library('refills')).


:- use_module(library('db/tripledb'), 
              [tripledb_load/2, tripledb_load/1]).

:- tripledb_load(
    'package://knowrob_refills/owl/shop.owl',
    [ namespace(shop, 
      'http://knowrob.org/kb/shop.owl#')
    ]).

:- tripledb_load(
    'package://knowrob_refills/owl/dm-market.owl',
    [ namespace(dmshop, 
      'http://knowrob.org/kb/dm-market.owl#')
    ]).


:- tripledb_load(
    'package://knowrob_refills/owl/iai-shop.owl',
    [ namespace(iaishop, 
      'http://knowrob.org/kb/iai-shop.owl#')
    ]).

% :- tripledb_load(
%     'package://knowrob/owl/robots/donbot_0418.owl',
%     [ namespace(donbot, 
%       'http://knowrob.org/kb/donbot.owl#')
%     ]).


:- tripledb_load('package://knowrob_refills/owl/product-catalog.owl').
:- tripledb_load('package://knowrob_refills/owl/product-taxonomy.owl').

% :- owl_parser:owl_parse('package://knowrob_refills/owl/shop.owl').
% :- owl_parser:owl_parse('package://knowrob_refills/owl/dm-market.owl').
% :- owl_parser:owl_parse('package://knowrob_refills/owl/product-taxonomy.owl').
% :- owl_parser:owl_parse('package://knowrob_refills/owl/product-catalog.owl').

% :- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
% :- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).
% :- rdf_db:rdf_register_ns(iaishop, 'http://knowrob.org/kb/iai-shop.owl#', [keep(true)]).
% :- rdf_db:rdf_register_ns(donbot, 'http://knowrob.org/kb/donbot.owl#', [keep(true)]).
