uint64 reasoner_id
flaw[] flaws
resolver[] resolvers
uint64 flaw_id
uint64 resolver_id
uint8 graph_changed = 0
uint8 flaw_created = 1
uint8 flaw_state_changed = 2
uint8 flaw_cost_changed = 3
uint8 flaw_position_changed = 4
uint8 current_flaw = 5
uint8 resolver_created = 6
uint8 resolver_state_changed = 7
uint8 current_resolver = 8
uint8 causal_link_added = 9
uint8 update