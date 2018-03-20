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
%    - use qudt and ensure unit is meters here.
%    - objects should have relation to the facing they are in, this needs
%      to be updated when separators/labels/.. are detected.
%    - potentially there will be problems if label perception is drastically
%        messed up, when the shelf structure is asserted first time.
%        i.e., if this messes up ordering, because at the moment it is expected that ordering
%        never changes once perceived.
%        should add a check if asserted spatial relations of re-preceived parts still hold
%    - include information about how much space can be taken by objects in layers

:- module(shop,
    [
      shelf_layer_above/2,
      shelf_layer_below/2,
      shelf_layer_mounting/1,
      shelf_layer_standing/1,
      shelf_layer_position/3,
      shelf_layer_find_nearest/5,
      shelf_layer_facings/2,
      shelf_layer_mounting_bars/2,
      shelf_facing/2,
      shelf_facing_previous/2,
      shelf_facing_next/2,
      shelf_facing_product_type/2,
      shelf_facing_total_space/2,
      shelf_facing_free_space/2,
      shelf_facing_occupied_space/2,
      shelf_mounting_bar/2,
      shelf_mounting_bar_previous/2,
      shelf_mounting_bar_next/2,
      shelf_label/2,
      shelf_label_previous/2,
      shelf_label_next/2,
      shelf_separator/2,
      % belief state updates
      belief_shelf_separator_at/3,
      belief_shelf_mounting_bar_at/3,
      belief_shelf_label_at/4,
      belief_shelf_product_at/4,
      % computable properties
      comp_isSpaceRemainingInFacing/2,
      comp_facingPose/2,
      comp_facingWidth/2,
      comp_facingHeight/2,
      comp_facingDepth/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).

:-  rdf_meta
    shelf_layer_above(r,r),
    shelf_layer_below(r,r),
    shelf_layer_mounting(r),
    shelf_layer_standing(r),
    shelf_layer_position(r,r,-),
    shelf_layer_find_nearest(r,+,t,r,-),
    shelf_layer_facings(r,-),
    shelf_layer_mounting_bars(r,-),
    shelf_facing(r,r),
    shelf_facing_previous(r,r),
    shelf_facing_next(r,r),
    shelf_facing_product_type(r,r),
    shelf_facing_total_space(r,?),
    shelf_facing_free_space(r,-),
    shelf_facing_occupied_space(r,-),
    shelf_mounting_bar(r,r),
    shelf_mounting_bar_previous(r,r),
    shelf_mounting_bar_next(r,r),
    shelf_label(r,r),
    shelf_label_previous(r,r),
    shelf_label_next(r,r),
    shelf_separator(r,r),
    belief_shelf_separator_at(r,t,-),
    belief_shelf_mounting_bar_at(r,t,-),
    belief_shelf_label_at(r,+,t,-),
    belief_shelf_product_at(r,r,t,-),
    comp_facingFree(r, ?).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

% TODO: should be somewhere else
linked_part(Part, Linked) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', Aff1),
  rdf_has(Conn, knowrob_assembly:consumesAffordance, Aff1),
  rdf_has(Conn, knowrob_assembly:needsAffordance, Aff2),
  Aff1 \= Aff2,
  rdf_has(Linked, knowrob_assembly:'hasAffordance', Aff2).

% TODO: should be somewhere else
% TODO: must work in both directions
xsd_float(Value, literal(
    type('http://www.w3.org/2001/XMLSchema#float', Atom))) :-
  atom(Value) -> Atom=Value ; atom_number(Atom,Value).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ArticleNumber'

shop_ean_article_number(EAN, ArticleNumber) :-
  rdf_has(ArticleNumber, shop:ean, literal(
    type('http://www.w3.org/2001/XMLSchema#string', EAN))), !.
shop_ean_article_number(EAN, ArticleNumber) :-
  owl_instance_from_class(shop:'EuropeanArticleNumber',ArticleNumber),
  rdf_assert(ArticleNumber, shop:ean, literal(
    type('http://www.w3.org/2001/XMLSchema#string', EAN))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLayer'

%% 
shelf_layer_above(ShelfLayer, AboveLayer) :-
  shelf_layer_top_affordance(ShelfLayer, Aff),
  ( rdfs_individual_of(Aff, shop:'ShelfBottomLayerAffordanceM') -> (
    shelf_layer_frame(ShelfLayer, ShelfFrame),
    shelf_frame_first_affordance(ShelfFrame, FirstAffordance),
    shelf_layer_above_(FirstAffordance, AboveLayer)
  );(
    shelf_layer_above_(Aff, AboveLayer)
  )), !.

shelf_layer_above_(Aff, AboveLayer) :-
  rdf_has(Aff, shop:nextShelfAffordance, NextAff),
  ( shelf_affordance_layer(NextAff, NextLayer) ->
    AboveLayer = NextLayer ;
    shelf_layer_above_(NextAff, AboveLayer) ), !.

%% 
shelf_layer_below(ShelfLayer, BelowLayer) :-
  shelf_layer_above(BelowLayer, ShelfLayer), !.
    
shelf_layer_frame(Layer, Frame) :-
  rdf_has(Layer, knowrob_assembly:hasAffordance, X),
  rdf_has(Conn, knowrob_assembly:consumesAffordance, X),
  rdf_has(Conn, knowrob_assembly:consumesAffordance, Y),
  X \= Y,
  rdf_has(Frame, knowrob_assembly:hasAffordance, Y),
  rdfs_individual_of(Frame, shop:'ShelfFrame').
    
shelf_frame_first_affordance(Frame, Affordance) :-
  rdf_has(Frame, knowrob_assembly:hasAffordance, Affordance),
  \+ rdf_has(_, shop:nextShelfAffordance, Affordance).

shelf_affordance_layer(Aff1, Layer) :-
  rdf_has(Conn, knowrob_assembly:consumesAffordance, Aff1),
  rdf_has(Conn, knowrob_assembly:consumesAffordance, Aff2),
  rdfs_individual_of(Aff2, shop:'ShelfLayerSlideInM'),
  rdf_has(Layer, knowrob_assembly:hasAffordance, Aff2).

shelf_layer_used_affordances(ShelfLayer, ConsumedAffordances) :-
  findall(Aff, (
    rdf_has(ShelfLayer, knowrob_assembly:hasAffordance, LayerAffordance),
    rdf_has(Conn, knowrob_assembly:consumesAffordance, LayerAffordance),
    rdf_has(Conn, knowrob_assembly:consumesAffordance, Aff)
  ), ConsumedAffordances).

shelf_layer_bottom_affordance(ShelfLayer, ShelfAffordance) :-
  shelf_layer_used_affordances(ShelfLayer, ConsumedAffordances),
  member(ShelfAffordance, ConsumedAffordances),
  \+ ( member(X, ConsumedAffordances),
       rdf_has(X, shop:nextShelfAffordance, ShelfAffordance)).

shelf_layer_top_affordance(ShelfLayer, ShelfAffordance) :-
  shelf_layer_used_affordances(ShelfLayer, ConsumedAffordances),
  member(ShelfAffordance, ConsumedAffordances),
  \+ ( member(X, ConsumedAffordances),
       rdf_has(ShelfAffordance, shop:nextShelfAffordance, X)).

%%
shelf_layer_distance(Layer1, Layer2, Distance) :-
  belief_at_relative_to(Layer1, Layer2, [_,_,[X,Y,Z],_]),
  Distance is sqrt(X*X + Y*Y + Z*Y).
    

%% shelf_layer_position
%
% The position of some object on a shelf layer.
% Position is simply the y-value of the object's pose in the shelf layer's frame.
%
shelf_layer_position(Layer, Object, Position) :-
  belief_at_relative_to(Object, Layer, [_,_,[_,Position,_],_]).
  
%%
shelf_layer_nearest_facing(ShelfLayer, Position, Nearest) :-
  shelf_layer_facings(ShelfLayer, FacingsSorted),
  shelf_layer_find_nearest(ShelfLayer, Position, FacingsSorted, Nearest, _).

%%
shelf_layer_find_nearest(ShelfLayer, _, [X], X, Pos_X) :-
  shelf_layer_position(ShelfLayer, X, Pos_X), !.

shelf_layer_find_nearest(ShelfLayer, Position, [X|Xs], Nearest, NearestPos) :-
  % TODO: use binary search
  % TODO: what is the origin of objects? potentially + 0.5*width or so to translate to center of object
  shelf_layer_position(ShelfLayer, X, Pos_X),
  ( Pos_X > Position -> (
    Nearest=X, NearestPos=Pos_X );(
    shelf_layer_find_nearest(ShelfLayer, Position, Xs, Y, Pos_Y),
    Diff_X is abs(Pos_X-Position),
    Diff_Y is abs(Pos_Y-Position),
    ( Diff_Y < Diff_X -> (
      Nearest=Y, NearestPos=Pos_Y );(
      Nearest=X, NearestPos=Pos_X ))
  )).

%% shelf_layer_mounting
shelf_layer_mounting(ShelfLayer) :-
  rdfs_individual_of(ShelfLayer, shop:'ShelfLayerMounting').
%% shelf_layer_standing
shelf_layer_standing(ShelfLayer) :-
  rdfs_individual_of(ShelfLayer, shop:'ShelfLayerStanding').

shelf_layer_update_labels(ShelfLayer) :-
  % step through all labels, find surrounding faces corresponding
  % to this label and associate them to the article number
  findall(FacingGroup, (
    shelf_label(ShelfLayer,Label),
    rdf_has(Facing, shop:labelOfFacing, Label),
    shelf_labeled_facings(LabeledFacing, [LeftFacings,RightFacings]),
    shelf_facings_update_label(LeftFacings, Label),
    shelf_facings_update_label(RightFacings, Label),
    append(LeftFacings, [LabeledFacing|RightFacings], FacingGroup)
  ), FacingGroups),
  flatten(FacingGroups, LabeledFacings),
  % retract article number for all remaining (orphan) facings
  forall(
    shelf_facing(ShelfLayer,Facing), (
    once((
      member(Facing,LabeledFacings) ;
      rdf_retractall(Facing, shop:labelOfFacing, _)
    ))
  )).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfSeparator'

%%
shelf_separator(ShelfLayer, Separator) :-
  shelf_layer_standing(ShelfLayer),
  linked_part(ShelfLayer, Separator),
  rdfs_individual_of(Separator, shop:'ShelfSeparator').

%%
shelf_separator_insert(ShelfLayer,Separator) :-
  shelf_layer_standing(ShelfLayer),
  % create connection between shelf layer and separator
  assemblage_connection_create(shop:'SeparatorConnectedToShelf', [ShelfLayer,Separator], _),
  shelf_layer_position(ShelfLayer, Separator, SeparatorPos),
  % try to find the facing cut into two pieces by Separator
  ( shelf_layer_nearest_facing(ShelfLayer,SeparatorPos,Facing) -> (
    rdf_has(Facing, shop:leftSeparator, Left),
    rdf_has(Facing, shop:rightSeparator, Right),
    shelf_facing_retract(Facing),
    shelf_facing_assert(ShelfLayer,[Left,Separator],_),
    shelf_facing_assert(ShelfLayer,[Separator,Right],_) );
  % else try to find another separator
  ((shelf_separator(ShelfLayer, OtherSeparator), OtherSeparator \= Separator) -> (
    shelf_layer_position(ShelfLayer, OtherSeparator, Pos_other),
    ( Pos_other < SeparatorPos ->
      shelf_facing_assert(ShelfLayer,[OtherSeparator,Separator],_) ;
      shelf_facing_assert(ShelfLayer,[Separator,OtherSeparator],_)
    )
  ) ; true)),
  shelf_layer_update_labels(ShelfLayer).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfMountingBar'

%%
shelf_mounting_bar(ShelfLayer, MountingBar) :-
  shelf_layer_mounting(ShelfLayer),
  linked_part(ShelfLayer, MountingBar),
  rdfs_individual_of(MountingBar, shop:'ShelfMountingBar').

shelf_mounting_bar_insert(ShelfLayer,MountingBar) :-
  shelf_layer_mounting(ShelfLayer),
  shelf_layer_position(ShelfLayer, MountingBar, MountingBarPos),
  shelf_mounting_bars_at(ShelfLayer, MountingBarPos, [Left,Right]),
  shelf_facing_assert(ShelfLayer,MountingBar,_),
  % create list structure between mounting MountingBars
  ( Left=none -> true ; (
    rdf_retractall(Left, shop:nextMountingBar, _),
    rdf_assert(Left, shop:nextMountingBar, MountingBar),
    rdf_assert(MountingBar, shop:previousMountingBar, Left) )),
  ( Right=none -> true ; (
    rdf_retractall(Right, shop:previousMountingBar, _),
    rdf_assert(Right, shop:previousMountingBar, MountingBar),
    rdf_assert(MountingBar, shop:nextMountingBar, Right) )),
  % create connection between shelf layer and mounting bar
  assemblage_connection_create(shop:'MountingBarConnectedToShelf', [ShelfLayer,MountingBar], _),
  % update the mounting_bar-label association
  shelf_layer_update_labels(ShelfLayer).

shelf_mounting_bars_at(ShelfLayer,Pos,[Left,Right]) :-
  shelf_layer_mounting_bars(ShelfLayer, MountingBars),
  shelf_layer_find_nearest(ShelfLayer, Pos, MountingBars, Nearest, NearestPos),
  ( NearestPos < Pos -> (
    % nearest is left of
    Left = Nearest, once((shelf_mounting_bar_next(Left,Right);Right=none))
  );(
    % nearest is right of
    Right = Nearest, once((shelf_mounting_bar_previous(Right,Left);Left=none))
  )).

%%
shelf_layer_mounting_bars(ShelfLayer, MountingBars) :-
  shelf_layer_leftmost_mounting_bar(ShelfLayer, LeftMost),
  shelf_layer_mounting_bars_(LeftMost,MountingBars).
shelf_layer_mounting_bars_(MountingBar, [Next|Rest]) :-
  shelf_mounting_bar_next(MountingBar,Next), !,
  shelf_layer_mounting_bars_(Next,Rest).
shelf_layer_mounting_bars_(_, []).

%%
shelf_layer_leftmost_mounting_bar(ShelfLayer, LeftMost) :-
  shelf_mounting_bar(ShelfLayer, MountingBar),!,
  shelf_layer_leftmost_mounting_bar_(MountingBar, LeftMost).
shelf_layer_leftmost_mounting_bar_(MountingBar, LeftMost) :-
  shelf_mounting_bar_previous(MountingBar,Previous), !,
  shelf_layer_leftmost_mounting_bar_(Previous,LeftMost).
shelf_layer_leftmost_mounting_bar_(MountingBar, MountingBar).

%%
shelf_mounting_bar_previous(MountingBar,Previous) :-
  rdf_has(MountingBar, shop:previousMountingBar, Previous).

%%
shelf_mounting_bar_next(MountingBar,Next) :-
  rdf_has(MountingBar, shop:nextMountingBar, Next).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ProductFacing'

%%
shelf_facing(ShelfLayer, Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer).

shelf_facing_assert(ShelfLayer,[Left,Right],Facing) :-
  shelf_layer_standing(ShelfLayer), !,
  rdfs_individual_of(Left, shop:'ShelfSeparator'),
  rdfs_individual_of(Right, shop:'ShelfSeparator'),
  owl_instance_from_class(shop:'ProductFacing',Facing),
  rdf_assert(Facing, shop:leftSeparator, Left),
  rdf_assert(Facing, shop:rightSeparator, Right),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer).

shelf_facing_assert(ShelfLayer,MountingBar,Facing) :-
  shelf_layer_mounting(ShelfLayer), !,
  rdfs_individual_of(MountingBar, shop:'ShelfMountingBar'),
  owl_instance_from_class(shop:'ProductFacing',Facing),
  rdf_assert(Facing, shop:mountingBarOfFacing, MountingBar),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer).

shelf_facing_retract(Facing) :-
  rdf_retractall(Facing, _, _).

shelf_facings_between(F, F, []) :- !.
shelf_facings_between(Left, Right, Between) :-
  shelf_facing_next(Left, Next),
  ( Next = Right -> Between = [] ; (
    shelf_facings_between(Next, Right,Rest),
    Between = [Next|Rest]
  )).

shelf_facings_before(Facing, LeftToRight) :-
  shelf_facings_before_(Facing, RightToLeft),
  reverse(RightToLeft, LeftToRight).
shelf_facings_before_(Facing, [Left|Rest]) :-
  shelf_facing_previous(Facing, Left),
  shelf_facings_before_(Left, Rest), !.
shelf_facings_before_(_, []).
  
shelf_facings_after(Facing, [Right|Rest]) :-
  shelf_facing_next(Facing, Right),
  shelf_facings_after(Right, Rest), !.
shelf_facings_after(_, []).

%%
shelf_facing_previous(Facing, Prev) :-
  rdf_has(Facing, shop:leftSeparator, X),
  rdf_has(Prev, shop:rightSeparator, X), !.
shelf_facing_previous(Facing, Prev) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar),
  rdf_has(MountingBar, shop:previousMountingBar, X),
  rdf_has(Prev, shop:mountingBarOfFacing, X).

%%
shelf_facing_next(Facing, Next) :-
  rdf_has(Facing, shop:rightSeparator, X),
  rdf_has(Next, shop:leftSeparator, X), !.
shelf_facing_next(Facing, Prev) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar),
  rdf_has(MountingBar, shop:nextMountingBar, X),
  rdf_has(Prev, shop:mountingBarOfFacing, X).
  

%% shelf_layer_facings
%
% Sorted list of facings visible on ShelfLayer.
% Ordered from left-to-right.
%
shelf_layer_facings(ShelfLayer, FacingsSorted) :-
  shelf_layer_standing(ShelfLayer), !,
  shelf_layer_leftmost_facing(ShelfLayer, Leftmost),
  shelf_layer_facings(ShelfLayer, Leftmost, FacingsSorted).

shelf_layer_facings(ShelfLayer, LeftFacing, [LeftFacing|Rest]) :-
  shelf_facing_next(LeftFacing, RightFacing),
  shelf_layer_facings(ShelfLayer, RightFacing, Rest), !.
shelf_layer_facings(_, Facing, [Facing]).

shelf_layer_leftmost_facing(ShelfLayer, Leftmost) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_leftmost_facing_(Facing, Leftmost).
shelf_layer_leftmost_facing_(Facing, Leftmost) :-
  rdf_has(Facing, shop:leftSeparator, Left),
  ( rdf_has(LeftFacing, shop:rightSeparator, Left) ->
    shelf_layer_leftmost_facing_(LeftFacing, Leftmost) ;
    Leftmost = Facing
  ).

%% shelf_facing_total_space
%
shelf_facing_total_space(Facing, Size) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  % TODO: not accurate! there is some space that can not be used by products!
  %       extend the owl model with this information
  owl_has(Facing, knowrob:depthOfObject, Depth), !,
  strip_literal_type(Depth, SizeAtom),
  atom_number(SizeAtom, Size).

%% shelf_facing_occupied_space
%
shelf_facing_occupied_space(Facing, OccupiedSpace) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  findall(Size, (
    rdf_has(Facing, shop:productInFacing, Product),
    object_dimensions(Product, _, _, Size)
  ), Sizes),
  sumlist(Sizes, OccupiedSpace).

%% shelf_facing_free_space
%
shelf_facing_free_space(Facing, Remaining) :-
  shelf_facing_total_space(Facing, Total),
  shelf_facing_occupied_space(Facing, Occupied),
  Remaining is Total - Occupied.

%% shelf_facing_product_type
%
shelf_facing_product_type(Facing, ProductType) :-
  owl_has(Facing, shop:articleNumberOfFacing, ArticleNumber),
  owl_class_properties(ProductType, shop:articleNumberOfProduct, ArticleNumber), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLabel'

shelf_label(ShelfLayer, Label) :-
  linked_part(ShelfLayer, Label),
  rdfs_individual_of(Label, shop:'ShelLabel').

shelf_label_insert(ShelfLayer,EAN,Label) :-
  % create connection between shelf layer and separator
  assemblage_connection_create(shop:'LabelConnectedToShelf', [ShelfLayer,Label], _),
  % assert the corresponding article number
  shop_ean_article_number(EAN, ArticleNumber),
  rdf_assert(Label, shop:articleNumberOfLabel, ArticleNumber),
  % find the position of Label on the shelf
  shelf_layer_position(ShelfLayer, Label, LabelPos),
  % first find the facing under which the label was perceived, 
  % then assert labelOfFacing and articleNumberOfFacing
  ( shelf_layer_nearest_facing(ShelfLayer,LabelPos,LabeledFacing) -> (
    rdf_retractall(LabeledFacing, shop:labelOfFacing, _),
    rdf_retractall(LabeledFacing, shop:articleNumberOfFacing, _),
    rdf_assert(LabeledFacing, shop:labelOfFacing, Label),
    rdf_assert(LabeledFacing, shop:articleNumberOfFacing, ArticleNumber)
  ) ; true),
  % update the facing-label relation
  shelf_layer_update_labels(ShelfLayer).

shelf_labeled_facings(LabeledFacing, [LeftScope,RightScope]) :-
  % the scope of labels is influenced by how far away the next label 
  % is to the left and right. The facings in between are evenly distributed between
  % the adjacent labels.
  ( shelf_label_previous(LabeledFacing, LeftLabel) -> (
    rdf_has(PrevFacing, shop:labelOfFacing, LeftLabel),
    shelf_facings_between(PrevFacing,LabeledFacing,Facings),
    length(Facings, NumFacings), Count is round(Numfacing / 2),
    take_tail(Facings,NumFacings,LeftFacings ) ;
    shelf_facings_before(LabeledFacing, LeftFacings)
  )),
  ( shelf_label_next(LabeledFacing, RightLabel) -> (
    rdf_has(NextFacing, shop:labelOfFacing, RightLabel),
    shelf_facings_between(LabeledFacing,NextFacing,Facings),
    length(Facings, NumFacings), Count is round(Numfacing / 2),
    take_head(Facings,NumFacings,RightFacings ) ;
    shelf_facings_after(LabeledFacing, RightFacings)
  )),
  % number of facings to the left and right which are understood to be labeled 
  % by Label must be evenly distributed, and identical in number to the left and
  % right of the label.
  length(LeftFacings, Left_count),
  length(RightFacings, Right_count),
  Count is min(Left_count,Right_count),
  take_tail(LeftFacings,Count,LeftScope),
  take_head(RightFacings,Count,RightScope).

shelf_facing_update_label(Facing, Label) :-
  rdf_has(Label, shop:articleNumberOfLabel, ArticleNumber),
  rdf_retractall(Facing, shop:articleNumberOfFacing, _),
  rdf_assert(Facing, shop:articleNumberOfFacing, ArticleNumber).

shelf_facings_update_label([], _) :- !.
shelf_facings_update_label([F|Rest], Label) :-
  shelf_facing_update_label(F, Label),
  shelf_facings_update_label(Rest, Label).

%%
shelf_label_previous(Facing, LeftLabel) :-
  shelf_facing_previous(Facing, LeftFacing),
  ( rdf_has(LeftFacing, shop:labelOfFacing, LeftLabel) ;
    shelf_label_previous(LeftFacing, LeftLabel) ), !.

%%
shelf_label_next(Facing, RightLabel) :-
  shelf_facing_next(Facing, RightFacing),
  ( rdf_has(RightFacing, shop:labelOfFacing, RightLabel) ;
    shelf_label_next(RightFacing, RightLabel) ), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Perception events

belief_shelf_frame_at(Transform, ShelfFrame) :- fail.

belief_shelf_layer_at(ShelfFrame, Transform, ShelfLayer) :- fail.

%% belief_shelf_separator_at
%
% Supposed to be called whenever the robot perceives one of the
% shelf separators.
% It must be aware of the ShelfLayer it is looking at.
%
belief_shelf_separator_at(ShelfLayer, Transform, Separator) :-
  shelf_layer_standing(ShelfLayer),
  belief_perceived_at_shelf_layer(ShelfLayer, shop:'ShelfSeparator', Transform, Separator),
  ( linked_part(ShelfLayer,Separator) ; 
    shelf_separator_insert(ShelfLayer,Separator)
  ), !.

belief_shelf_mounting_bar_at(ShelfLayer, Transform, MountingBar) :-
  shelf_layer_mounting(ShelfLayer),
  belief_perceived_at_shelf_layer(ShelfLayer, shop:'ShelfMountingBar', Transform, MountingBar),
  ( linked_part(ShelfLayer,MountingBar) ; 
    shelf_mounting_bar_insert(ShelfLayer,MountingBar)
  ), !.

belief_shelf_label_at(ShelfLayer, ArticleNumber, Transform, Label) :-
  belief_perceived_at_shelf_layer(ShelfLayer, shop:'ShelfLabel', Transform, Label),
  ( linked_part(ShelfLayer,Label) -> true ; (
    shelf_label_insert(ShelfLayer,ArticleNumber,Label)
  )).

belief_shelf_product_at(ShelfLayer, Type, Transform, Product) :-
  % TODO: relate object to the facing it belongs to, relation must be updated when stuff is perceived
  belief_perceived_at_shelf_layer(ShelfLayer, Type, Transform, Product).

belief_perceived_at_shelf_layer(_ShelfLayer, Type, Transform, Object) :-
  % TODO: enforce that pose relative to ShelfLayer, would avoid later conversion
  belief_perceived_at(Type, Transform, Object).
  

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% COMPUTABLE shop:'isSpaceRemainingInFacing'

%% comp_isSpaceRemainingInFacing
%
comp_isSpaceRemainingInFacing(Facing,
  literal(type('http://www.w3.org/2001/XMLSchema#boolean', true))) :-
  shelf_facing_free_space(Facing, Remaining),
  ( shelf_facing_product_type(Facing, ProductType) -> (
    object_class_dimensions(ProductType, _, _, ProductSize),
    Remaining > ProductSize );
    true % assume space is remaining if the facing does not have an associated product type
  ), !.
comp_isSpaceRemainingInFacing(_,
  literal(type('http://www.w3.org/2001/XMLSchema#boolean', false))).

%% comp_facingPose
%
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar), !,
  % FIXME: pose needs to have an offset according to facing width
  current_object_pose(MountingBar, Pose).
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:leftSeparator, Separator), !,
  current_object_pose(Separator, Pose).

%% comp_facingWidth
%
comp_facingWidth(Facing, Width) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  Width_value is Pos_Right - Pos_Left,
  xsd_float(Width_value, Width).
comp_facingWidth(Facing, Width) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar),
  rdf_has(MountingBar, shop:leftMountingBar, Left),
  rdf_has(MountingBar, shop:rightMountingBar, Right),
  % just assume facings evenly share the space.
  % might not be entirely accureate but good enough, I guess
  shelf_layer_position(ShelfLayer, Left, Pos0),
  shelf_layer_position(ShelfLayer, MountingBar, Pos1),
  shelf_layer_position(ShelfLayer, Right, Pos2),
  Width_value is min(Pos1 - Pos0, Pos2 - Pos1),
  xsd_float(Width_value, Width).

%% comp_facingDepth
%
comp_facingDepth(Facing, Depth) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  object_depth(ShelfLayer, Depth_value),
  xsd_float(Depth_value, Depth).

%% comp_facingHeight
%
comp_facingHeight(Facing, Height) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  shelf_layer_above(ShelfLayer, LayerAbove),
  shelf_layer_distance(ShelfLayer, LayerAbove, Distance),
  ( shelf_layer_standing(ShelfLayer) ->
    % above is also standing layer, whole space can be taken
    xsd_float(Distance, Height) ; (
    % above is mounting layer, space must be shared. For now assume equal space sharing
    Height_value is Distance * 0.5,
    xsd_float(Height_value, Height)
  )).
comp_facingHeight(Facing, Height) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  shelf_layer_below(ShelfLayer, LayerBelow),
  shelf_layer_distance(ShelfLayer, LayerBelow, Distance),
  ( shelf_layer_mounting(ShelfLayer) ->
    % above is also mounting layer, whole space can be taken
    xsd_float(Distance, Height) ; (
    Height_value is Distance * 0.5,
    xsd_float(Height_value, Height)
  )).
