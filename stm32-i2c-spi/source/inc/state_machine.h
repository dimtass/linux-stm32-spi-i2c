#ifndef STATE_MACHINE_
#define STATE_MACHINE_

#include <stdint.h>
#include "platform_config.h"


struct state_item_t;

#define DECLARE_STATE_MACHINE(NAME,STATE_LIST) \
    struct state_machine_t NAME = { \
        .state_list = STATE_LIST, \
        .state_curr = NULL, \
        .state_next = NULL, \
    }


struct state_machine_t {
    struct state_item_t *state_curr;
    struct state_item_t *state_next;
    struct state_item_t *state_list;
};


#define DECLARE_STATE_ITEM(STATE,DATA,ENTER,RUN,EXIT) { \
            .state = STATE, \
            .data = DATA, \
            .enter = ENTER, \
            .run = RUN, \
            .exit = EXIT, \
        }

struct state_item_t {
    uint8_t     state;
    void        *data;
    void (*enter)(uint8_t prev_state);
    void (*run)();
    void (*exit)(uint8_t next_state);
};

static inline void init_state_machine(struct state_machine_t * sm, uint8_t start_state)
{
    /* Force to run the state enter */
    sm->state_curr = sm->state_next = &sm->state_list[start_state];
    if (sm->state_next->enter)
        sm->state_next->enter(sm->state_curr->state);
    if (sm->state_curr->run)
        sm->state_curr->run();
}


static inline void state_change(struct state_machine_t * sm, uint8_t next_state)
{
    sm->state_next = &sm->state_list[next_state];
}

static inline void state_handler(struct state_machine_t * sm)
{
    if (sm->state_curr != sm->state_next) {
        // TRACE(("state: %d -> %d\n", sm->state_curr->state, sm->state_next->state));
        if (sm->state_curr->exit)
            sm->state_curr->exit(sm->state_next->state);
        if (sm->state_next->enter)
            sm->state_next->enter(sm->state_curr->state);
        sm->state_curr = sm->state_next;
    }
    if (sm->state_curr->run)
        sm->state_curr->run();
}

static inline uint8_t state_get_curr(struct state_machine_t * sm)
{
    return sm->state_curr->state;
}


#endif //STATE_MACHINE_