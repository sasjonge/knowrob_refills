/**  <module> shop

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

% TODO:
%    - add conversation hook for ean/dan?
%    - use qudt and ensure unit is meters here.
%    - include information about how much space can be taken by objects in layers

:- module(shop,
    [
      shelf_layer_frame/2,
      shelf_layer_mounting/1,
      shelf_layer_standing/1,
      shelf_layer_above/2,
      shelf_layer_below/2,
      shelf_layer_position/3,
      shelf_layer_separator/2,
      shelf_layer_mounting_bar/2,
      shelf_layer_label/2,
      shelf_layer_find_facing_at/3,
      shelf_facing/2,
      shelf_facing_product_type/2,
      shelf_facings_mark_dirty/1,
      shelf_separator_insert/2,
      shelf_separator_insert/3,
      shelf_label_insert/2,
      shelf_label_insert/3,
      % computable properties
      comp_isSpaceRemainingInFacing/2,
      comp_facingPose/2,
      comp_facingWidth/2,
      comp_facingHeight/2,
      comp_facingDepth/2,
      comp_preferredLabelOfFacing/2,
      %%%%% rooming in
      belief_shelf_left_marker_at/3,
      belief_shelf_right_marker_at/3,
      belief_shelf_at/3,
      shelf_classify/4,
      shelf_with_marker/2,
      %shelf_estimate_pose/1,
      %%%%%
      belief_shelf_part_at/4,
      belief_shelf_part_at/5,
      belief_shelf_barcode_at/5,
      belief_shelf_barcode_at/6,
      product_spawn_front_to_back/2,
      product_spawn_front_to_back/3,
      %%%%%
      create_article_type/2,
      create_article_type/3,
      create_article_number/3,
      create_article_number/2,
      article_number_of_dan/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/knowrob')).

:-  rdf_meta
    shelf_layer_frame(r,r),
    shelf_layer_above(r,r),
    shelf_layer_below(r,r),
    shelf_layer_mounting(r),
    shelf_layer_standing(r),
    shelf_layer_position(r,r,-),
    shelf_layer_mounting_bar(r,r),
    shelf_layer_label(r,r),
    shelf_layer_separator(r,r),
    shelf_facing_product_type(r,r),
    shelf_layer_part(r,r,r),
    belief_shelf_part_at(r,r,+,r),
    belief_shelf_part_at(r,r,+,r,+),
    belief_shelf_barcode_at(r,r,+,+,-),
    belief_shelf_barcode_at(r,r,+,+,-,+),
    product_type_dimension_assert(r,r,+),
    product_spawn_front_to_back(r,r,r),
    product_spawn_front_to_back(r,r),
    shelf_classify(r,+,+,+),
    shelf_with_marker(r,r),
    rdfs_classify(r,r),
    owl_classify(r,r),
    shelf_facings_mark_dirty(r),
    create_article_type(r,r).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).

% TODO: should be somewhere else
% TODO: must work in both directions
xsd_float(Value, literal(
    type('http://www.w3.org/2001/XMLSchema#float', Atom))) :-
  atom(Value) -> Atom=Value ; atom_number(Atom,Value).
xsd_boolean(Atom, literal(
    type('http://www.w3.org/2001/XMLSchema#boolean', Atom))) :-
  atom(Atom).

prolog:message(shop(Entities,Msg)) -->
  %{ owl_readable(Entities,Entities_readable) },
  ['[shop.pl] ~w ~w'-[Msg,Entities]].
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ArticleNumber'

%create_article_number(ean(AN), ArticleNumber) :-
  %create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#ean',AN)), ArticleNumber).
%create_article_number(dan(AN), ArticleNumber) :-
  %create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#dan',AN)), ArticleNumber).
%create_article_number_(AN_XSD,ArticleNumber) :-
  %owl_has(ArticleNumber, shop:articleNumberString, AN_XSD), !.
%create_article_number_(AN_XSD, ArticleNumber) :-
  %strip_literal_type(AN_XSD, AN_atom),
  %atomic_list_concat([
    %'http://knowrob.org/kb/shop.owl#ArticleNumber_',
    %AN_atom], ArticleNumber),
  %rdf_assert(ArticleNumber, rdf:type, shop:'ArticleNumber'),
  %rdf_assert(ArticleNumber, rdf:type, owl:'NamedIndividual'),
  %rdf_assert(ArticleNumber, shop:articleNumberString, AN_XSD),
  %create_product_type(ArticleNumber, ProductType),
  %print_message(warning, shop([ProductType], 'Missing product type. Incomplete data?')).
  
atomize(A,A) :- atom(A), !.
atomize(T,A) :- term_to_atom(T,A).

create_article_number(GTIN,DAN,AN) :-
  atomize(GTIN,GTIN_atom),
  atomize(DAN,DAN_atom),
  kb_create(shop:'ArticleNumber', AN, _{ graph: belief_state }),
  rdf_assert(AN, shop:gtin, literal(type(xsd:string,GTIN_atom)), belief_state),
  rdf_assert(AN, shop:dan, literal(type(xsd:string,DAN_atom)), belief_state).

create_article_number(dan(DAN),AN) :-
  atomize(DAN,DAN_atom),
  kb_create(shop:'ArticleNumber', AN, _{ graph: belief_state }),
  rdf_assert(AN, shop:dan, literal(type(xsd:string,DAN_atom)), belief_state).

create_article_number(gtin(GTIN),AN) :-
  atomize(GTIN,GTIN_atom),
  kb_create(shop:'ArticleNumber', AN, _{ graph: belief_state }),
  rdf_assert(AN, shop:gtin, literal(type(xsd:string,GTIN_atom)), belief_state).

create_article_type(AN,[D,W,H],ProductType) :-
  create_article_type(AN,ProductType),
  % specify bounding box
  xsd_float(D,D_XSD),
  xsd_float(W,W_XSD),
  xsd_float(H,H_XSD),
  owl_restriction_assert(restriction(
    shop:widthOfProduct, has_value(W_XSD)), W_R, belief_state),
  owl_restriction_assert(restriction(
    shop:heightOfProduct, has_value(H_XSD)), H_R, belief_state),
  owl_restriction_assert(restriction(
    shop:depthOfProduct, has_value(D_XSD)), D_R, belief_state),
  rdf_assert(ProductType, rdfs:subClassOf, W_R, belief_state),
  rdf_assert(ProductType, rdfs:subClassOf, H_R, belief_state),
  rdf_assert(ProductType, rdfs:subClassOf, D_R, belief_state).

create_article_type(AN,ProductType) :-
  once((
    kb_triple(AN,shop:gtin,NUM) ;
    kb_triple(AN,shop:dan,NUM) )),
  atomic_list_concat([
    'http://knowrob.org/kb/shop.owl#UnknownProduct_',NUM], ProductType),
  rdf_assert(ProductType, rdfs:subClassOf, shop:'Product', belief_state),
  rdf_assert(ProductType, rdf:type, owl:'Class', belief_state),
  owl_restriction_assert(restriction(
    shop:articleNumberOfProduct, has_value(AN)), AN_R, belief_state),
  rdf_assert(ProductType, rdfs:subClassOf, AN_R, belief_state).

article_number_of_dan(DAN,AN) :-
  kb_triple(AN,shop:dan,DAN),
  rdfs_individual_of(AN,shop:'ArticleNumber'),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'Product'

product_type_dimensions([D,W,H], [D,W,H]) :- !.
product_type_dimensions(Type, [D,W,H]) :-
  owl_property_range_on_class(Type, shop:depthOfProduct,  literal(type(_,D_atom))),
  owl_property_range_on_class(Type, shop:widthOfProduct,  literal(type(_,W_atom))),
  owl_property_range_on_class(Type, shop:heightOfProduct, literal(type(_,H_atom))),
  atom_number(D_atom, D),
  atom_number(W_atom, W),
  atom_number(H_atom, H), !.
product_type_dimensions(Type, [0.04,0.04,0.04]) :-
  print_message(warning, shop(Type,'No bounding box is defined')),
  product_type_dimension_assert(Type, shop:depthOfProduct, 0.04),
  product_type_dimension_assert(Type, shop:widthOfProduct, 0.04),
  product_type_dimension_assert(Type, shop:heightOfProduct, 0.04), !.

product_type_dimension_assert(Type, P, Val) :-
  xsd_float(Val,Val_XSD),
  owl_restriction_assert(restriction(P, has_value(Val_XSD)), R),
  rdf_assert(Type, rdfs:subClassOf, R),
  owl:owl_property_range_clear_cache(Type,P).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLayer'

%% 
shelf_layer_mounting(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerMounting').
%% 
shelf_layer_standing(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerStanding').
shelf_layer_standing_bottom(ShelfLayer) :- rdfs_individual_of(ShelfLayer, dmshop:'DMShelfBFloor'), !.

%% 
shelf_layer_frame(Layer, Frame) :-
  owl_has(Frame, knowrob:properPhysicalParts, Layer),
  rdfs_individual_of(Frame, shop:'ShelfFrame'), !.
%%
shelf_facing(ShelfLayer, Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer).

%%
shelf_layer_separator(Layer,Part)    :- shelf_layer_part(Layer,shop:'ShelfSeparator',Part).
%%
shelf_layer_mounting_bar(Layer,Part) :- shelf_layer_part(Layer,shop:'ShelfMountingBar',Part).
%%
shelf_layer_label(Layer,Part)        :- shelf_layer_part(Layer,shop:'ShelfLabel',Part).
%%
shelf_layer_part(Layer, Type, Part) :-
  rdfs_individual_of(Layer, shop:'ShelfLayer'),
  owl_has(Layer, knowrob:properPhysicalParts, Part),
  rdfs_individual_of(Part, Type).

%% shelf_layer_position
%
% The position of some object on a shelf layer.
% Position is simply the x-value of the object's pose in the shelf layer's frame.
%
shelf_layer_position(Layer, Object, Position) :-
  object_frame_name(Layer, LayerFrame),
  object_pose(Object, [LayerFrame,_,[Position,_,_],_]).

%%
shelf_facing_position(Facing,Pos) :-
  rdf_has(Facing, shop:leftSeparator, Left), !,
  rdf_has(Facing, shop:rightSeparator, Right),
  rdf_has(Facing, shop:layerOfFacing, Layer),
  shelf_layer_position(Layer, Left, Pos_Left),
  shelf_layer_position(Layer, Right, Pos_Right),
  Pos is 0.5*(Pos_Left+Pos_Right).
shelf_facing_position(Facing, Pos) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  shelf_layer_position(Layer, MountingBar, Pos).

%% 
shelf_layer_above(ShelfLayer, AboveLayer) :-
  shelf_layer_sibling(ShelfLayer, min_positive_element, AboveLayer).
%% 
shelf_layer_below(ShelfLayer, BelowLayer) :-
  shelf_layer_sibling(ShelfLayer, max_negative_element, BelowLayer).

shelf_layer_sibling(ShelfLayer, Selector, SiblingLayer) :-
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  object_frame_name(ShelfFrame, ShelfFrameName),
  object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,Pos],_]),
  findall((X,Diff), (
    rdf_has(ShelfFrame, knowrob:properPhysicalParts, X),
    X \= ShelfLayer,
    object_pose(X, [ShelfFrameName,_,[_,_,X_Pos],_]),
    Diff is X_Pos-Pos), Xs),
  call(Selector, Xs, (SiblingLayer,_)).

shelf_layer_neighbours(ShelfLayer, Needle, Selector, Positions) :-
  shelf_layer_position(ShelfLayer, Needle, NeedlePos),
  findall((X,D), (
    call(Selector, ShelfLayer, X),
    X \= Needle,
    shelf_layer_position(ShelfLayer, X, Pos_X),
    D is Pos_X - NeedlePos
  ), Positions).

shelf_facing_labels_update(ShelfLayer,Facing) :-
  forall((
    owl_has(Facing,shop:adjacentLabelOfFacing,Label);
    rdf_has(Facing,shop:labelOfFacing,Label)),(
    shelf_label_insert(ShelfLayer,Label)
  )).

shelf_facings_mark_dirty(ShelfLayer) :-
  findall(X, (
    shelf_facing(ShelfLayer,X),
    shelf_facing_update(X)
  ), AllFacings),
  mark_dirty_objects(AllFacings).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfSeparator'

%%
shelf_separator_insert(ShelfLayer,Separator) :-
  shelf_separator_insert(ShelfLayer,Separator,[update_facings]).
shelf_separator_insert(ShelfLayer,Separator,Options) :-
  shelf_layer_standing(ShelfLayer),
  % [X.pos - Separator.pos]
  shelf_layer_neighbours(ShelfLayer, Separator, shelf_layer_separator, Xs),
  ignore(max_negative_element(Xs, (LeftOf,_))),
  ignore(min_positive_element(Xs, (RightOf,_))),
  %
  (( rdf_has(_,shop:rightSeparator,Separator) ;
     rdf_has(_,shop:leftSeparator,Separator)) ->
     shelf_separator_update(ShelfLayer, Separator, [LeftOf,RightOf]);
     shelf_separator_add(   ShelfLayer, Separator, [LeftOf,RightOf]) ),
  % update labelOfFacing relation
  ((rdf_has(Y,shop:leftSeparator,Separator);
    rdf_has(Y,shop:rightSeparator,Separator)) ->
    shelf_facing_labels_update(ShelfLayer,Y) ; true ),
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).
   
shelf_separator_update(_, X, [LeftOf,RightOf]) :-
  % FIXME: could be that productInFacing changes, but highly unlikely.
  %        this is at the moment only updated when a facing is retracted.
  % LeftOf,RightOf unchanged if ...
  ( rdf_has(Facing1,shop:rightSeparator,X) ->
  ( ground(LeftOf), rdf_has(Facing1,shop:leftSeparator,LeftOf));    \+ ground(LeftOf)),
  ( rdf_has(Facing2,shop:leftSeparator,X) ->
  ( ground(RightOf), rdf_has(Facing2,shop:rightSeparator,RightOf)); \+ ground(RightOf)), !.
shelf_separator_update(Layer, X, [LeftOf,RightOf]) :-
  % LeftOf,RightOf changed otherwise
  shelf_separator_remove(X),
  shelf_separator_add(Layer, X, [LeftOf,RightOf]).

shelf_separator_add(Layer, X, [LeftOf,RightOf]) :-
  (ground(RightOf) -> shelf_facing_assert(Layer,[X,RightOf],_) ; true),
  (ground(LeftOf)  -> shelf_facing_assert(Layer,[LeftOf,X],_) ; true),
  ((ground([RightOf,LeftOf]),
    rdf_has(Facing, shop:rightSeparator,RightOf),
    rdf_has(Facing, shop:leftSeparator,LeftOf)) ->
    shelf_facing_retract(Facing) ; true ).

shelf_separator_remove(X) :-
  ( rdf_has(LeftFacing,shop:rightSeparator,X) ->
    shelf_separator_remove_rightSeparator(LeftFacing,X); true ),
  ( rdf_has(RightFacing,shop:leftSeparator,Y) ->
    shelf_separator_remove_leftSeparator(RightFacing,Y); true ).
shelf_separator_remove_rightSeparator(Facing,X) :-
  rdf_has(Facing,shop:leftSeparator,Left),
  rdf_has(NextFacing,shop:leftSeparator,X),
  rdf_retractall(NextFacing,shop:leftSeparator,X),
  rdf_assert(NextFacing,shop:leftSeparator,Left),
  shelf_facing_retract(Facing).
shelf_separator_remove_leftSeparator(Facing,X) :-
  rdf_has(Facing,shop:rightSeparator,Right),
  rdf_has(NextFacing,shop:rightSeparator,X),
  rdf_retractall(NextFacing,shop:rightSeparator,X),
  rdf_assert(NextFacing,shop:rightSeparator,Right),
  shelf_facing_retract(Facing).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfMountingBar'

shelf_mounting_bar_insert(ShelfLayer,MountingBar) :-
  shelf_mounting_bar_insert(ShelfLayer,MountingBar,[update_facings]).
shelf_mounting_bar_insert(ShelfLayer,MountingBar,Options) :-
  (( rdf_has(Facing,shop:mountingBarOfFacing,MountingBar),
     shelf_mounting_bar_remove(MountingBar) );
     shelf_facing_assert(ShelfLayer,MountingBar,Facing)), !,
  % [X.pos - MountingBar.pos]
  shelf_layer_neighbours(ShelfLayer, MountingBar, shelf_layer_mounting_bar, Xs),
  ( min_positive_element(Xs, (X,_)) -> (
    % positive means that X is right of Separator
    rdf_assert(Facing, shop:rightMountingBar, X, belief_state),
    rdf_has(RightFacing, shop:mountingBarOfFacing, X),
    rdf_retractall(RightFacing, shop:leftMountingBar, _),
    rdf_assert(RightFacing, shop:leftMountingBar, MountingBar, belief_state)) ;
    true ),
  ( max_negative_element(Xs, (Y,_)) -> (
    % negative means that Y is left of Separator
    rdf_assert(Facing, shop:leftMountingBar, Y, belief_state),
    rdf_has(LeftFacing, shop:mountingBarOfFacing, Y),
    rdf_retractall(LeftFacing, shop:rightMountingBar, _),
    rdf_assert(LeftFacing, shop:rightMountingBar, MountingBar, belief_state)) ;
    true ),
  % update labelOfFacing and productInFacing relations
  shelf_facing_labels_update(ShelfLayer,Facing),
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).

shelf_mounting_bar_remove(MountingBar) :-
  rdf_has(Facing,shop:mountingBarOfFacing,MountingBar),
  (( rdf_has(Facing, shop:rightMountingBar, Right),
     rdf_has(Facing, shop:leftMountingBar, Left) ) -> (
     rdf_has(RightFacing, shop:mountingBarOfFacing, Right),
     rdf_has(LeftFacing, shop:mountingBarOfFacing, Left),
     rdf_assert(RightFacing,shop:leftMountingBar,Left),
     rdf_assert(LeftFacing,shop:rightMountingBar,Right)
  );true),
  rdf_retractall(_,shop:leftMountingBar,MountingBar),
  rdf_retractall(_,shop:rightMountingBar,MountingBar).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLabel'

shelf_label_insert(ShelfLayer,Label) :-
  shelf_label_insert(ShelfLayer,Label,[update_facings]).
shelf_label_insert(ShelfLayer,Label,Options) :-
  rdf_retractall(_,shop:labelOfFacing,Label),
  % find the position of Label on the shelf
  shelf_layer_position(ShelfLayer, Label, LabelPos),
  kb_triple(Label, knowrob:widthOfObject, LabelWidth),
  LabelPosLeft  is LabelPos - 0.5*LabelWidth,
  LabelPosRight is LabelPos + 0.5*LabelWidth,
  % first find the facing under which the label was perceived, 
  % then assert labelOfFacing relation
  ( shelf_layer_find_facing_at(ShelfLayer,LabelPosLeft,LabeledFacingLeft) ->
    rdf_assert(LabeledFacingLeft,shop:labelOfFacing,Label,belief_state) ;
    true ),
  ((shelf_layer_find_facing_at(ShelfLayer,LabelPosRight,LabeledFacingRight),
    LabeledFacingRight \= LabeledFacingLeft ) ->
    rdf_assert(LabeledFacingRight,shop:labelOfFacing,Label,belief_state) ;
    true ),
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ProductFacing'

shelf_facing_assert(ShelfLayer,[Left,Right],Facing) :-
  shelf_layer_standing(ShelfLayer), !,
  kb_create(shop:'ProductFacingStanding', Facing, _{ graph: belief_state }),
  rdf_assert(Facing, shop:leftSeparator, Left, belief_state),
  rdf_assert(Facing, shop:rightSeparator, Right, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state).

shelf_facing_assert(ShelfLayer,MountingBar,Facing) :-
  shelf_layer_mounting(ShelfLayer), !,
  kb_create(shop:'ProductFacingMounting', Facing, _{ graph: belief_state }),
  rdf_assert(Facing, shop:mountingBarOfFacing, MountingBar, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state).

shelf_facing_retract(Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  rdf_retractall(Facing, shop:layerOfFacing, _),
  % remove products from facing
  forall(rdf_has(Product,shop:productInFacing,Facing),
         shelf_layer_insert_product(ShelfLayer,Product)),
  % TODO: don't forget about this facing, also marker remve msg should be generated
  rdf_retractall(Facing, _, _).

shelf_layer_insert_product(ShelfLayer,Product) :-
  shelf_layer_position(ShelfLayer,Product,Pos_Product),
  ( shelf_layer_find_facing_at(ShelfLayer,Pos_Product,Facing) ->
    rdf_assert(Facing, shop:productInFacing, Product, belief_state) ;
    true ).

shelf_layer_find_facing_at(ShelfLayer,Pos,Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_facing_position(Facing,FacingPos),
  shelf_facing_width(Facing, FacingWidth),
  Width is FacingWidth+0.02,
  FacingPos-0.5*Width =< Pos, Pos =< FacingPos+0.5*Width, !.

facing_space_remaining_in_front(Facing,Obj) :-
  object_pose(Obj, [_,_,[_,Obj_pos,_],_]),
  product_dimensions(Obj,Obj_depth,_,_),
  object_dimensions(Facing,Facing_depth,_,_),
  Obj_pos > Obj_depth + 0.06 - 0.5*Facing_depth.

facing_space_remaining_behind(Facing,Obj) :-
  object_pose(Obj, [_,_,[_,Obj_pos,_],_]),
  product_dimensions(Obj,Obj_depth,_,_),
  object_dimensions(Facing,Facing_depth,_,_),
  Obj_pos < Facing_depth*0.5 - Obj_depth - 0.06.

%% shelf_facing_product_type
%
shelf_facing_product_type(Facing, ProductType) :-
  comp_preferredLabelOfFacing(Facing,Label),
  rdf_has(Label,shop:articleNumberOfLabel,ArticleNumber),
  rdf_has(R, owl:hasValue, ArticleNumber),
  %rdf_has(R, owl:onProperty, shop:dan),
  rdf_has(ProductType, rdfs:subClassOf, R),
  rdf_has(ProductType, rdf:type, owl:'Class'), !.
shelf_facing_product_type(Facing, _) :-
  print_message(warning, shop([Facing], 'Facing has no associated product type.')),
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% COMPUTABLES

%% comp_preferredLabelOfFacing
%
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing),
  % preferred if label is one of the labelOfFacing
  findall(L,rdf_has(Facing,shop:labelOfFacing,L),[X|Xs]), !,
  member(Label,[X|Xs]).
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing), !,
  owl_has(Facing,shop:adjacentLabelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  rdf_has(Facing,shop:labelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  % preferred if label is adjacent to facing without labelOfFacing
  owl_has(Facing,shop:adjacentLabelOfFacing,Label),
  \+ rdf_has(Facing,shop:labelOfFacing,_).

%% comp_isSpaceRemainingInFacing
%
comp_isSpaceRemainingInFacing(Facing,Val_XSD) :-
  shelf_facing_products(Facing, ProductsFrontToBack),
  ((ProductsFrontToBack=[] ; (
    reverse(ProductsFrontToBack, ProductsBackToFront),
    ProductsFrontToBack=[(_,First)|_],
    ProductsBackToFront=[(_,Last)|_], (
    facing_space_remaining_in_front(Facing,First);
    facing_space_remaining_behind(Facing,Last))
  )) ->
  xsd_boolean('true',Val_XSD);
  xsd_boolean('false',Val_XSD)),!.

shelf_facing_update(Facing) :-
  % update geometry
  comp_facingDepth(Facing,D),
  comp_facingWidth(Facing,W),
  comp_facingHeight(Facing,H),
  object_assert_dimensions(Facing,D,W,H),
  % update pose
  comp_facingPose(Facing,Pose),
  object_pose_update(Facing,Pose),
  % preferred label
  rdf_retractall(Facing,shop:preferredLabelOfFacing,Label),
  ( comp_preferredLabelOfFacing(Facing,Label) ->
    rdf_assert(Facing,shop:preferredLabelOfFacing,Label) ;
    true
  ),
  % update color
  comp_mainColorOfFacing(Facing,Color),
  object_assert_color(Facing,Color).

%% comp_facingPose
%
comp_facingPose(Facing, [FloorFrame,FacingFrame,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0]]) :-
  rdf_has(Facing, shop:leftSeparator, _), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  object_frame_name(Facing,FacingFrame),
  object_frame_name(Layer,FloorFrame),
  object_dimensions(Facing, _, _, Facing_H),
  shelf_facing_position(Facing, Pos_X),
  Pos_Y is -0.02,               % 0.06 to leave some room at the front and back of the facing
  % FIXME: should be done by offset, also redundant with spawn predicate
  ( shelf_layer_standing_bottom(Layer) ->
    Pos_Z is 0.5*Facing_H + 0.025 ;
    Pos_Z is 0.5*Facing_H + 0.08 ).
comp_facingPose(Facing, [FloorFrame,FacingFrame,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0]]) :-
  rdf_has(Facing, shop:mountingBarOfFacing, _), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  object_frame_name(Facing,FacingFrame),
  object_frame_name(Layer,FloorFrame),
  comp_facingHeight(Facing, Facing_H),
  shelf_facing_position(Facing,Pos_X),
  Pos_Y is -0.03,           
  Pos_Z is -0.5*Facing_H.
  
%% comp_facingWidth
%
comp_facingWidth(Facing, Value) :-
  shelf_facing_width(Facing,Value).

shelf_facing_width(Facing, Value) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  Value is abs(Pos_Right - Pos_Left) - 0.02. % leave 2cm to each side
shelf_facing_width(Facing, Value) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar),
  shelf_layer_position(ShelfLayer, MountingBar, MountingBarPos),
  object_dimensions(ShelfLayer, _, LayerWidth, _),
  ( rdf_has(Facing, shop:leftMountingBar, Left) ->
    shelf_layer_position(ShelfLayer, Left, LeftPos) ;
    LeftPos is -0.5*LayerWidth
  ),
  ( rdf_has(Facing, shop:rightMountingBar, Right) ->
    shelf_layer_position(ShelfLayer, Right, RightPos) ;
    RightPos is 0.5*LayerWidth
  ),
  Value is min(MountingBarPos - LeftPos,
               RightPos - MountingBarPos). % leave 1cm to each side

%% comp_facingHeight
%
comp_facingHeight(Facing, Value) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  object_frame_name(ShelfFrame, ShelfFrameName),
  object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_above(ShelfLayer, LayerAbove) -> (
    object_pose(LayerAbove, [ShelfFrameName,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer above
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H - X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_standing(LayerAbove) -> % FIXME could be unbound
    % above is also standing layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % above is mounting layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ), !.
comp_facingHeight(Facing, Value) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  object_frame_name(ShelfFrame, ShelfFrameName),
  object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_below(ShelfLayer, LayerBelow) -> (
    object_pose(LayerBelow, [ShelfFrameName,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer below
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H + X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_mounting(LayerBelow) ->  % FIXME could be unbound
    % below is also mounting layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % below is standing layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ).

%% comp_facingDepth
%
comp_facingDepth(Facing, Value) :- comp_facingDepth(Facing, shelf_layer_standing, -0.06, Value).
comp_facingDepth(Facing, Value) :- comp_facingDepth(Facing, shelf_layer_mounting, 0.0, Value).
comp_facingDepth(Facing, Selector, Offset, Value) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  call(Selector, ShelfLayer), !,
  object_dimensions(ShelfLayer, Value0, _, _),
  Value is Value0 + Offset.

comp_mainColorOfFacing(Facing, Color) :-
  rdf_has(Facing, shop:layerOfFacing, _), !,
  ((kb_type_of(Facing, shop:'UnlabeledProductFacing'),Color=[1.0, 0.35, 0.0, 0.5]);
   % FIXME: MisplacedProductFacing seems slow, comp_MisplacedProductFacing is a bit faster
   (comp_MisplacedProductFacing(Facing),Color=[1.0, 0.0, 0.0, 0.5]);
   %(kb_type_of(Facing, shop:'MisplacedProductFacing'),Color=[1.0, 0.0, 0.0, 0.5]);
   (kb_type_of(Facing, shop:'EmptyProductFacing'),Color=[1.0, 1.0, 0.0, 0.5]);
   (kb_type_of(Facing, shop:'FullProductFacing'),Color=[0.0, 0.25, 0.0, 0.5]);
   Color=[0.0, 1.0, 0.0, 0.5]), !.

comp_MisplacedProductFacing(Facing) :-
  rdf_has(Facing,shop:productInFacing,Product),
  owl_has(Product,shop:articleNumberOfProduct,AN),
  forall( rdf_has(Label,shop:articleNumberOfLabel,AN),
          \+ comp_preferredLabelOfFacing(Facing,Label) ),!.
comp_MisplacedProductFacing(Facing) :-
  rdf_has(Facing,shop:productInFacing,Product),
  \+ owl_has(Product,shop:articleNumberOfProduct,_),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% "rooming in" -- perceive shelf barcodes and classify shelves
% TODO: make marker pose relative
% TODO: shelf pose computation

shelf_marker_offset(0.04).

shelf_type(LeftMarker,RightMarker,ShelfType) :-
  ValidWidths=[
    [0.65,'http://knowrob.org/kb/dm-market.owl#DMShelfW60'],
    [0.73,'http://knowrob.org/kb/dm-market.owl#DMShelfW70'],
    [1.0,'http://knowrob.org/kb/dm-market.owl#DMShelfW100'],
    [1.2,'http://knowrob.org/kb/dm-market.owl#DMShelfW120']
  ],
  % estimate width from markers
  object_distance(LeftMarker,RightMarker,Distance),
  shelf_marker_offset(Offset),
  WidthEstimate is Distance + Offset,
  % find closest match
  findall([D,Type], (
    member([Width,Type],ValidWidths),
    D is abs(WidthEstimate - Width)),
    ShelfTypes),
  sort(ShelfTypes, [[_,ShelfType]|_]).

shelf_with_marker(Shelf,Marker) :- (
  rdf_has(Shelf,dmshop:leftMarker,Marker);
  rdf_has(Shelf,dmshop:rightMarker,Marker)),!.
shelf_with_marker(Shelf,Id) :-
  atom(Id),
  kb_triple(Marker,dmshop:markerId,Id),
  shelf_with_marker(Shelf,Marker),!.

shelf_marker(Id,Marker):-
  atom(Id), kb_triple(Marker,dmshop:markerId,Id),!.
shelf_marker(Marker,Marker):-
  atom(Marker), kb_triple(Marker,dmshop:markerId,_),!.

%%
%
belief_shelf_marker_at(MarkerType,MarkerId,Pose,Marker):-
  belief_new_object(MarkerType, Marker),
  kb_assert(Marker, dmshop:markerId, MarkerId),
  belief_at_update(Marker,Pose).
%%
%
belief_shelf_left_marker_at(Pose,MarkerId,Marker):-
  rdf_equal(MarkerType,dmshop:'DMShelfMarkerLeft'),
  belief_shelf_marker_at(MarkerType,MarkerId,Pose,Marker).
%%
%
belief_shelf_right_marker_at(Pose,MarkerId,Marker):-
  rdf_equal(MarkerType,dmshop:'DMShelfMarkerRight'),
  belief_shelf_marker_at(MarkerType,MarkerId,Pose,Marker).

%%
%
belief_shelf_at(LeftMarkerId,RightMarkerId,Shelf) :-
  shelf_marker(LeftMarkerId,LeftMarker),
  shelf_marker(RightMarkerId,RightMarker),
  once((
    % asserted before
    shelf_with_marker(Shelf,LeftMarker);
    shelf_with_marker(Shelf,RightMarker);
    % new shelf
    belief_new_shelf_at(LeftMarkerId,RightMarkerId,Shelf))).

belief_new_shelf_at(LeftMarkerId,RightMarkerId,Shelf) :-
  % infer shelf type (e.g. 'DMShelfFrameW100')
  shelf_marker(LeftMarkerId,LeftMarker),
  shelf_marker(RightMarkerId,RightMarker),
  shelf_type(LeftMarker,RightMarker,ShelfType),
  % assert to belief state and link marker to shelf
  belief_new_object(ShelfType,Shelf),
  % temporary assert height/depth
  % FIXME: what is going on here? why assert? rather use object_* predicates
rdfs_classify(Shelf,ShelfType),
  rdf_assert(Shelf, knowrob:depthOfObject, literal(type(xsd:float, '0.02'))),
  rdf_assert(Shelf, knowrob:heightOfObject, literal(type(xsd:float, '0.11'))),
  rdf_assert(Shelf, knowrob:mainColorOfObject, literal(type(xsd:string, '0.0 1.0 0.5 0.6'))),
  rdf_assert(Shelf, dmshop:leftMarker, LeftMarker, belief_state),
  rdf_assert(Shelf, dmshop:rightMarker, RightMarker, belief_state).

shelf_find_type(Shelf,Type) :-
  rdfs_subclass_of(Type,dmshop:'DMShelfFrame'),
  forall((
    rdf_has(Shelf,rdf:type,X),
    rdfs_subclass_of(X,dmshop:'DMShelfFrame')),
    owl_subclass_of(Type,X)).

%%
%
shelf_classify(Shelf,Height,NumTiles,Payload) :-
  % retract temporary height/depth
  rdf_retractall(Shelf, knowrob:depthOfObject, _),
  rdf_retractall(Shelf, knowrob:heightOfObject, _),
  rdf_retractall(Shelf, knowrob:mainColorOfObject, _),
  % assert various types based on input.
  % this fails in case input is not valid.
  shelf_classify_height(Shelf,Height),
  shelf_classify_num_tiles(Shelf,NumTiles),
  shelf_classify_payload(Shelf,Payload),
  % use closed world semantics to infer all the shelf frame
  % types currently implied for `Shelf`
  ( shelf_find_type(Shelf,ShelfType) -> (
    print_message(info, shop([Shelf,ShelfType], 'Is classified as.')),
    rdfs_classify(Shelf,ShelfType));(
    findall(X,(
      rdf_has(Shelf,rdf:type,X),
      rdfs_subclass_of(X,dmshop:'DMShelfFrame')),Xs),
    print_message(warning, shop([Shelf,Xs], 'Failed to classify. Type not defined in ontology?'))
  )).

%%
% Classify shelf based on its height.
shelf_classify_height(Shelf,1.6):-
  rdfs_classify(Shelf, dmshop:'DMShelfH160'),!.
shelf_classify_height(Shelf,1.8):-
  rdfs_classify(Shelf, dmshop:'DMShelfH180'),!.
shelf_classify_height(Shelf,2.0):-
  rdfs_classify(Shelf, dmshop:'DMShelfH200'),!.
shelf_classify_height(_Shelf,H):-
  print_message(warning, shop([H], 'Is not a valid shelf height (one of [1.6,1.8,2.0]).')),
  fail.

%%
% Classify shelf based on number of tiles in bottom floor.
shelf_classify_num_tiles(Shelf,4):-
  rdfs_classify(Shelf, dmshop:'DMShelfT4'),!.
shelf_classify_num_tiles(Shelf,5):-
  rdfs_classify(Shelf, dmshop:'DMShelfT5'),!.
shelf_classify_num_tiles(Shelf,6):-
  rdfs_classify(Shelf, dmshop:'DMShelfT6'),!.
shelf_classify_num_tiles(Shelf,7):-
  rdfs_classify(Shelf, dmshop:'DMShelfT7'),!.
shelf_classify_num_tiles(_Shelf,Count):-
  print_message(warning, shop([Count], 'Is not a valid number of shelf tiles (one of [5,6,7]).')),
  fail.

%%
% Classify shelf based on payload.
shelf_classify_payload(Shelf,heavy) :-
  rdfs_classify(Shelf, dmshop:'DMShelfH'),!.
shelf_classify_payload(Shelf,light) :-
  rdfs_classify(Shelf, dmshop:'DMShelfL'),!.
shelf_classify_payload(_Shelf,Mode):-
  print_message(warning, shop([Mode], 'Is not a valid payload mode (one of [heavy,light]).')),
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% belief_state part of shelves

%%
%
% This predicate exists to establish some relations
% between labels and facings, and to create facings
% between separators and mounting bars.
%
belief_shelf_part_at(Frame, Type, Pos, Obj) :-
  belief_shelf_part_at(Frame, Type, Pos, Obj, [insert,update_facings]).

belief_shelf_part_at(Frame, Type, Pos, Obj, _Options) :-
  rdfs_subclass_of(Type, shop:'ShelfLayer'), !,
  pos_term(y,Pos,PosTerm),
  perceived_part_at_axis__(Frame, Type, PosTerm, Obj),
  % adding a new shelf floor has influence on facing size of
  % shelf floor siblings
  ( shelf_layer_below(Obj,Below) ->
    shelf_facings_mark_dirty(Below) ; true ),
  ( shelf_layer_above(Obj,Above) ->
    shelf_facings_mark_dirty(Above) ; true ).

belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :-
  rdfs_subclass_of(Type, shop:'ShelfSeparator'), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_separator_insert(Layer,Obj,Options) ;
    true ).

belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :-
  rdfs_subclass_of(Type, shop:'ShelfMountingBar'), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_mounting_bar_insert(Layer,Obj,Options) ;
    true ).

belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :-
  rdfs_subclass_of(Type, shop:'ShelfLabel'), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_label_insert(Layer,Obj,Options) ;
    true ).

%%
%%

perceived_pos__([DX,DY,DZ], pos(x,P), [X,DY,DZ]) :- X is DX+P, !.
perceived_pos__([DX,DY,DZ], pos(z,P), [DX,Y,DZ]) :- Y is DY+P, !.
perceived_pos__([DX,DY,DZ], pos(y,P), [DX,DY,Z]) :- Z is DZ+P, !.

denormalize_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_), Out is V*In.
denormalize_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V), Out is V*In.
denormalize_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_), Out is V*In.

center_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_),
  Out is In - 0.5*V.
center_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V),
  Out is In - 0.5*V.
center_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_),
  Out is In - 0.5*V.

belief_part_offset(Parent, PartType, Offset, Rotation) :-
  object_disposition(Parent,Linkage,DispositionType),
  once(owl_subclass_of(DispositionType,ease_obj:'Linkage')),
  disposition_trigger_type(Linkage,PartType),
  kb_triple(Linkage,ease_obj:hasSpaceRegion,LinkageSpace),
  transform_data(LinkageSpace,(Offset, Rotation)),!.
belief_part_offset(_, _, [0,0,0], [0,0,0,1]).

perceived_part_at_axis__(Parent, PartType, norm(Axis,Pos), Part) :- !,
  denormalize_part_pos(Parent, Axis, Pos, Denormalized),
  perceived_part_at_axis__(Parent, PartType, pos(Axis,Denormalized), Part).

perceived_part_at_axis__(Parent, PartType, pos(Axis,Pos), Part) :-
  center_part_pos(Parent, Axis, Pos, Centered),
  object_frame_name(Parent,ParentFrame),
  belief_part_offset(Parent, PartType, Offset, Rotation),
  perceived_pos__(Offset, pos(Axis,Centered), PerceivedPos),
  belief_perceived_part_at(PartType, [ParentFrame,_,PerceivedPos,
      Rotation], 0.02, Part, Parent).

%%
%%

belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj) :-
  belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj, [insert,update_facings]).

belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj, Options) :-
  %create_article_number(ArticleNumber_value, ArticleNumber),
  belief_shelf_part_at(Layer, Type, PosNorm, Obj, Options),
  % 
  forall( article_number_of_dan(DAN,AN),
          rdf_assert(Obj, shop:articleNumberOfLabel, AN, belief_state) ).

pos_term(Axis, norm(Pos), norm(Axis,Pos)) :- !.
pos_term(Axis, Pos, pos(Axis,Pos)).

product_dimensions(X,D,W,H):-object_dimensions(X,D,W,H), !.
product_dimensions(_,0.04,0.04,0.04).

product_spawn_at(Facing, TypeOrBBOX, Offset_D, Obj) :-
  rdf_has(Facing, shop:layerOfFacing, Layer),
  
  product_type_dimensions(TypeOrBBOX, [Obj_D,_,Obj_H]),
  object_dimensions(Layer,Layer_D,_,_),
  Layer_D*0.5 > Offset_D + Obj_D*0.5 + 0.04,
  
  ( TypeOrBBOX=[D,W,H] -> (
    belief_new_object(shop:'Product', Obj),
    object_assert_dimensions(Obj,D,W,H) ) ;
    belief_new_object(TypeOrBBOX, Obj) ),
  % enforce we have a product here
  ( rdfs_individual_of(Obj,shop:'Product') -> true ;(
    print_message(warning, shop([Obj], 'Is not subclass of shop:Product.')),
    rdf_assert(Obj,rdf:type,shop:'Product') )),
  
  % compute offset
  %product_dimensions(Obj,_,_,Obj_H),
  object_pose(Facing, [_,_,[Facing_X,_,_],_]),
  
  % FIXME: this should be handled by offsets from ontology
  ( shelf_layer_standing(Layer) -> (
    shelf_layer_standing_bottom(Layer) ->
    Offset_H is  Obj_H*0.5 + 0.025 ;
    Offset_H is  Obj_H*0.5 + 0.08 ) ;
    Offset_H is -Obj_H*0.5 - 0.025 ),
  
  % HACK rotate if it has a mesh
  ( object_mesh_path(Obj,_) ->
    Rot=[0.0, 0.0, 1.0, 0.0] ;
    Rot=[0.0, 0.0, 0.0, 1.0] ),
  %Rot=[0.0, 0.0, 0.0, 1.0],
  
  % declare transform
  object_frame_name(Layer, Layer_frame),
  belief_at_update(Obj, [Layer_frame,_, 
      [Facing_X, Offset_D, Offset_H],
      Rot]),
  rdf_assert(Facing, shop:productInFacing, Obj, belief_state),
  
  % update facing
  comp_mainColorOfFacing(Facing,Color),
  object_assert_color(Facing,Color),
  mark_dirty_objects([Facing]).

product_spawn_front_to_back(Facing, Obj) :-
  shelf_facing_product_type(Facing, ProductType),
  product_spawn_front_to_back(Facing, Obj, ProductType).
  
product_spawn_front_to_back(Facing, Obj, TypeOrBBOX) :-
  rdf_has(Facing, shop:layerOfFacing, Layer),
  product_type_dimensions(TypeOrBBOX, [Obj_D,_,_]),
  shelf_facing_products(Facing, ProductsFrontToBack),
  reverse(ProductsFrontToBack, ProductsBackToFront),
  ( ProductsBackToFront=[] -> (
    object_dimensions(Layer,Layer_D,_,_),
    Obj_Pos is -Layer_D*0.5 + Obj_D*0.5 + 0.01,
    product_spawn_at(Facing, TypeOrBBOX, Obj_Pos, Obj));(
    ProductsBackToFront=[(Last_Pos,Last)|_],
    product_dimensions(Last,Last_D,_,_),
    Obj_Pos is Last_Pos + 0.5*Last_D + 0.5*Obj_D + 0.02,
    product_spawn_at(Facing, TypeOrBBOX, Obj_Pos, Obj)
  )).
  
shelf_facing_products(Facing, Products) :-
  findall((Pos,Product), (
    rdf_has(Facing, shop:productInFacing, Product),
    current_object_pose(Product, [_,_,[_,Pos,_],_])), Products_unsorted),
  sort(Products_unsorted, Products).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Helper predicates

max_negative_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A > 0.0, !, max_negative_element(Xs, (Needle,D_Needle)).
max_negative_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  max_negative_element(Rest, (B,D_B)),
  ( D_A > D_B -> (
    Needle=A, D_Needle=D_A );(
    Needle=B, D_Needle=D_B )).
max_negative_element([(A,D_A)|_], (A,D_A)).

min_positive_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A < 0.0, !, min_positive_element(Xs, (Needle,D_Needle)).
min_positive_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  min_positive_element(Rest, (B,D_B)),
  ( D_A < D_B ->
    Needle=A, D_Needle=D_A;
    Needle=B, D_Needle=D_B ).
min_positive_element([(A,D_A)|_], (A,D_A)).

rdfs_classify(Entity,Type) :-
  forall((
    rdf_has(Entity,rdf:type,X),
    rdfs_subclass_of(Type,X)),
    rdf_retractall(Entity,rdf:type,X)),
  rdf_assert(Entity,rdf:type,Type,belief_state).

owl_classify(Entity,Type) :-
  % find RDF graph of Entity
  forall((
    rdf_has(Entity,rdf:type,X),
    once(owl_subclass_of(Type,X))),
    rdf_retractall(Entity,rdf:type,X)),
  rdf_assert(Entity,rdf:type,Type,belief_state).
