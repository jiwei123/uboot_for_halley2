#ifndef _BILIST_H_
#define _BILIST_H_
/*
  from linux / list.h
 */

struct bilist_head {
	struct bilist_head *next, *prev;
};
#define BILIST_HEAD_INIT(name) { &(name), &(name) }

#define BILIST_HEAD(name) \
	struct bilist_head name = BILIST_HEAD_INIT(name)

static inline void INIT_BILIST_HEAD(struct bilist_head *list)
{
	list->next = list;
	list->prev = list;
}

static inline void __bilist_add(struct bilist_head *new,
			      struct bilist_head *prev,
			      struct bilist_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static inline void bilist_add(struct bilist_head *new, struct bilist_head *head)
{
	__bilist_add(new, head, head->next);
}
static inline void bilist_add_tail(struct bilist_head *new, struct bilist_head *head)
{
	__bilist_add(new, head->prev, head);
}
static inline void __bilist_del(struct bilist_head * prev, struct bilist_head * next)
{
	next->prev = prev;
	prev->next = next;
}

static inline void __bilist_del_entry(struct bilist_head *entry)
{
	__bilist_del(entry->prev, entry->next);
}

static inline void bilist_del(struct bilist_head *entry)
{
	__bilist_del(entry->prev, entry->next);
	entry->next = 0;
	entry->prev = 0;
}

static inline void bilist_replace(struct bilist_head *old,
				struct bilist_head *new)
{
	new->next = old->next;
	new->next->prev = new;
	new->prev = old->prev;
	new->prev->next = new;
}

static inline void bilist_replace_init(struct bilist_head *old,
					struct bilist_head *new)
{
	bilist_replace(old, new);
	INIT_BILIST_HEAD(old);
}
static inline void bilist_del_init(struct bilist_head *entry)
{
	__bilist_del_entry(entry);
	INIT_BILIST_HEAD(entry);
}
static inline void bilist_move(struct bilist_head *list, struct bilist_head *head)
{
	__bilist_del_entry(list);
	bilist_add(list, head);
}
static inline void bilist_move_tail(struct bilist_head *list,
				  struct bilist_head *head)
{
	__bilist_del_entry(list);
	bilist_add_tail(list, head);
}
static inline int bilist_is_last(const struct bilist_head *list,
				const struct bilist_head *head)
{
	return list->next == head;
}

static inline int bilist_empty(const struct bilist_head *head)
{
	return head->next == head;
}
static inline int bilist_empty_careful(const struct bilist_head *head)
{
	struct bilist_head *next = head->next;
	return (next == head) && (next == head->prev);
}

static inline void bilist_rotate_left(struct bilist_head *head)
{
	struct bilist_head *first;

	if (!bilist_empty(head)) {
		first = head->next;
		bilist_move_tail(first, head);
	}
}
static inline int bilist_is_singular(const struct bilist_head *head)
{
	return !bilist_empty(head) && (head->next == head->prev);
}

static inline void __bilist_cut_position(struct bilist_head *list,
		struct bilist_head *head, struct bilist_head *entry)
{
	struct bilist_head *new_first = entry->next;
	list->next = head->next;
	list->next->prev = list;
	list->prev = entry;
	entry->next = list;
	head->next = new_first;
	new_first->prev = head;
}

static inline void bilist_cut_position(struct bilist_head *list,
		struct bilist_head *head, struct bilist_head *entry)
{
	if (bilist_empty(head))
		return;
	if (bilist_is_singular(head) &&
		(head->next != entry && head != entry))
		return;
	if (entry == head)
		INIT_BILIST_HEAD(list);
	else
		__bilist_cut_position(list, head, entry);
}

static inline void __bilist_splice(const struct bilist_head *list,
				 struct bilist_head *prev,
				 struct bilist_head *next)
{
	struct bilist_head *first = list->next;
	struct bilist_head *last = list->prev;

	first->prev = prev;
	prev->next = first;

	last->next = next;
	next->prev = last;
}

static inline void bilist_splice(const struct bilist_head *list,
				struct bilist_head *head)
{
	if (!bilist_empty(list))
		__bilist_splice(list, head, head->next);
}

static inline void bilist_splice_tail(struct bilist_head *list,
				struct bilist_head *head)
{
	if (!bilist_empty(list))
		__bilist_splice(list, head->prev, head);
}

static inline void bilist_splice_init(struct bilist_head *list,
				    struct bilist_head *head)
{
	if (!bilist_empty(list)) {
		__bilist_splice(list, head, head->next);
		INIT_BILIST_HEAD(list);
	}
}

static inline void bilist_splice_tail_init(struct bilist_head *list,
					 struct bilist_head *head)
{
	if (!bilist_empty(list)) {
		__bilist_splice(list, head->prev, head);
		INIT_BILIST_HEAD(list);
	}
}

#define bilist_offsetof(TYPE, MEMBER) ((unsigned int) &((TYPE *)0)->MEMBER)

#define bicontainer_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - bilist_offsetof(type,member) );})

#define bilist_entry(ptr, type, member) \
	bicontainer_of(ptr, type, member)

#define bilist_first_entry(ptr, type, member) \
	bilist_entry((ptr)->next, type, member)

#define bilist_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define __bilist_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define bilist_for_each_prev(pos, head) \
	for (pos = (head)->prev; pos != (head); pos = pos->prev)

#define bilist_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define bilist_for_each_prev_safe(pos, n, head) \
	for (pos = (head)->prev, n = pos->prev; \
	     pos != (head); \
	     pos = n, n = pos->prev)

#define bilist_for_each_entry(pos, head, member)				\
	for (pos = bilist_entry((head)->next, typeof(*pos), member);	\
	     &pos->member != (head); 	\
	     pos = bilist_entry(pos->member.next, typeof(*pos), member))

#define bilist_for_each_entry_reverse(pos, head, member)			\
	for (pos = bilist_entry((head)->prev, typeof(*pos), member);	\
	     &pos->member != (head); 	\
	     pos = bilist_entry(pos->member.prev, typeof(*pos), member))

#define bilist_prepare_entry(pos, head, member) \
	((pos) ? : bilist_entry(head, typeof(*pos), member))

#define bilist_for_each_entry_continue(pos, head, member) 		\
	for (pos = bilist_entry(pos->member.next, typeof(*pos), member);	\
	     &pos->member != (head);	\
	     pos = bilist_entry(pos->member.next, typeof(*pos), member))

#define bilist_for_each_entry_continue_reverse(pos, head, member)		\
	for (pos = bilist_entry(pos->member.prev, typeof(*pos), member);	\
	     &pos->member != (head);	\
	     pos = bilist_entry(pos->member.prev, typeof(*pos), member))

#define bilist_for_each_entry_from(pos, head, member) 			\
	for (; &pos->member != (head);	\
	     pos = bilist_entry(pos->member.next, typeof(*pos), member))

#define bilist_for_each_entry_safe(pos, n, head, member)			\
	for (pos = bilist_entry((head)->next, typeof(*pos), member),	\
		n = bilist_entry(pos->member.next, typeof(*pos), member);	\
	     &pos->member != (head); 					\
	     pos = n, n = bilist_entry(n->member.next, typeof(*n), member))

#define bilist_for_each_entry_safe_continue(pos, n, head, member) 		\
	for (pos = bilist_entry(pos->member.next, typeof(*pos), member), 		\
		n = bilist_entry(pos->member.next, typeof(*pos), member);		\
	     &pos->member != (head);						\
	     pos = n, n = bilist_entry(n->member.next, typeof(*n), member))

#define bilist_for_each_entry_safe_from(pos, n, head, member) 			\
	for (n = bilist_entry(pos->member.next, typeof(*pos), member);		\
	     &pos->member != (head);						\
	     pos = n, n = bilist_entry(n->member.next, typeof(*n), member))

#define bilist_for_each_entry_safe_reverse(pos, n, head, member)		\
	for (pos = bilist_entry((head)->prev, typeof(*pos), member),	\
		n = bilist_entry(pos->member.prev, typeof(*pos), member);	\
	     &pos->member != (head); 					\
	     pos = n, n = bilist_entry(n->member.prev, typeof(*n), member))

#define bilist_safe_reset_next(pos, n, member)				\
	n = bilist_entry(pos->member.next, typeof(*pos), member)

#endif /* _BILIST_H_ */
