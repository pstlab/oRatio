# notifies the state of a reasoner
uint64 reasoner_id
uint8 created = 0
uint8 reasoning = 1
uint8 inconsistent = 2
uint8 idle = 3
uint8 executing = 4
uint8 finished = 5
uint8 destroyed = 6
uint8 deliberative_state